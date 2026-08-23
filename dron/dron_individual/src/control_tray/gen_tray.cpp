#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>                      // Librería necesaria para crear la acción.

#include "dron_individual/action/tray_action.hpp"               // Añadir interfaz usada en la acción.
#include "lib_tray/gen_tray_pol3.hpp"                           // Librería polinomio cúbico
#include "lib_tray/gen_tray_veltrap.hpp"                        // Librería velocidad trapezoidal
#include "lib_tray/gen_tray_elipse.hpp"                         // Librería elipse

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>
#include <array>
#include <vector>
#include <memory>
#include <functional>
#include <exception>


class Clase_Servicio_Accion : public rclcpp::Node
{
    public:
        using TrayAction = dron_individual::action::TrayAction;
        using GoalHandleTrayAction = rclcpp_action::ServerGoalHandle<TrayAction>;

        Clase_Servicio_Accion() : rclcpp::Node("gen_tray")
        {
            // Suscripción (siempre activa). En execute solo usamos la primera muestra que llegue.
            sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "sensor/GT/pose", rclcpp::QoS(10), std::bind(&Clase_Servicio_Accion::pose_actual_callback, this, std::placeholders::_1));
            sub_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "sensor/GT/vel", rclcpp::QoS(10), std::bind(&Clase_Servicio_Accion::vel_actual_callback, this, std::placeholders::_1));

            // Añadimos al objeto del servidor de la acción el tipo de interfaz a utilizar, el nombre de la acción el cual
            // el cliente de la acción debe llamar para acceder a él, y el nombre de las función callback.
            objeto_servicio_accion = rclcpp_action::create_server<TrayAction>(this, "AccionTrayectoria",
                std::bind(&Clase_Servicio_Accion::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Clase_Servicio_Accion::handle_cancel, this, std::placeholders::_1),
                std::bind(&Clase_Servicio_Accion::handle_accepted, this, std::placeholders::_1));

            this->declare_parameter<double>("crear.tray.v_max_lin", 0.8);
            this->declare_parameter<double>("crear.tray.v_max_ang", 0.5);
            this->declare_parameter<double>("crear.tray.t_a", 2.0);

            v_max_lin = this->get_parameter("crear.tray.v_max_lin").as_double();
            v_max_ang = this->get_parameter("crear.tray.v_max_ang").as_double();
            t_a = this->get_parameter("crear.tray.t_a").as_double();
        }

    private:


        ///////////////// CALLBACKS /////////////////
        // Obtener Pose Actual
        void pose_actual_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(pose_callback_mtx_);
            if(!bloquear_callback)
            {
                last_pose_ = *msg;
                bloquear_callback = true;
                RCLCPP_INFO(this->get_logger(), "pose_actualizada (callback)");
            }
        }

        // Obtener Vel Actual
        void vel_actual_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(vel_callback_mtx_);
            if (!bloquear_vel)
            {
                last_vel_ = *msg;
                bloquear_vel = true;
                RCLCPP_INFO(this->get_logger(), "vel_actualizada (callback)");
            }
        }






        ///////////////// FUNCIONES /////////////////
        // Proyectar velocidad lineal máxima sobre cada eje
        double proyeccion(double velocidad_abs, char eje, double x0, double y0, double z0, double xf, double yf, double zf)
        {
            const double dx = xf - x0;
            const double dy = yf - y0;
            const double dz = zf - z0;

            const double L = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (L <= 1e-9)
            {
                return 0.0;
            }

            double comp = 0.0;

            if (eje == 'X') comp = std::abs(dx / L);
            else if (eje == 'Y') comp = std::abs(dy / L);
            else if (eje == 'Z') comp = std::abs(dz / L);

            double vel_eje = comp * velocidad_abs;

            return vel_eje;
        }

        // Obtener "Yaw" de la variable tipo "Pose"
        double pose2yaw(const geometry_msgs::msg::PoseStamped &pose_stamped)
        {
            double x = pose_stamped.pose.orientation.x;
            double y = pose_stamped.pose.orientation.y;
            double z = pose_stamped.pose.orientation.z;
            double w = pose_stamped.pose.orientation.w;

            double seno = 2.0 * (w * z + x * y);
            double coseno = 1.0 - 2.0 * (y * y + z * z);

            double yaw = std::atan2(seno, coseno);
            return yaw;
        }

        // Normalizar ángulo entre [-pi, pi]
        double normalizar_angulo(double angulo)
        {
            while (angulo > M_PI)
            {
                angulo -= 2.0 * M_PI;
            }

            while (angulo < -M_PI)
            {
                angulo += 2.0 * M_PI;
            }

            return angulo;
        }

        // Pasar de std::array a Float64MultiArray
        std_msgs::msg::Float64MultiArray array_to_msg(const std::array<double, 5> & valores)
        {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.assign(valores.begin(), valores.end());
            return msg;
        }








        ///////////////// FUNCIONES ACCIÓN /////////////////
        // Se llama cuando el cliente de la acción realiza la petición
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const TrayAction::Goal> goal)
        {
            if (goal->tipo_trayectoria > 2)
            {
                RCLCPP_WARN(this->get_logger(), "tipo_trayectoria no válido: %u", goal->tipo_trayectoria);
                return rclcpp_action::GoalResponse::REJECT;
            }

            // Si devolvemos (ACCEPT_AND_EXECUTE) se llama a la función "handle_accepted" (y se interrumpe la acción anterior)
            // Si devolvemos (ACCEPT_AND_DEFER) se espera a que termine la acción anterior y se llama a "handle_accepted"
            // Si devolvemos (REJECT) rechazamos el goal y no se llama a "handle_accepted"

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Se llama cuando el cliente cancela la acción (por ejemplo: goal_handle.cancel_goal_async()).
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTrayAction> /*goal_handle*/)
        {
            RCLCPP_INFO(this->get_logger(), "Petición de cancelación recibida");

            // Si devolvemos (ACCEPT), la acción se cancela.
            // Si devolvemos (REJECT), la acción sigue.
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Se llama cuando se acepta la petición en "handle_goal"
        void handle_accepted(const std::shared_ptr<GoalHandleTrayAction> goal_handle)
        {
            // Llama a la función "execute" como hilo (en segundo plano) con el fin de no bloquear la acción ante otras posibles llamadas
            std::shared_ptr<GoalHandleTrayAction> prev_goal;

            active_mtx_.lock();
            prev_goal = active_goal_;
            active_goal_ = goal_handle;
            active_mtx_.unlock();

            if (prev_goal && prev_goal->is_active())
            {
                auto res = std::make_shared<TrayAction::Result>();
                res->success = false;
                res->t_total = 0.0f;
                prev_goal->abort(res);
            }

            std::thread(&Clase_Servicio_Accion::execute, this, goal_handle).detach();
        }









        ///////////////// FUNCIÓN PRINCIPAL /////////////////
        void execute(const std::shared_ptr<GoalHandleTrayAction> goal_handle)
        {
            // Para que cada vez que se llame a la acción se reinicien las variables.
            bool eliminado = false;         // al declararse aquí, es variable local de cada thread
            bool cancelado = false;         // necesario para goal_handle->canceled
            pose_callback_mtx_.lock();
            bloquear_callback = false;
            pose_callback_mtx_.unlock();
            vel_callback_mtx_.lock();
            bloquear_vel = false;
            vel_callback_mtx_.unlock();
            double t = 0.0;

            rclcpp::Rate wait_rate(100.0);

            bool pose_ready = false, vel_ready = false;;        // creada para aislarla de la variable global "bloquear_callback"

            while (rclcpp::ok() && !eliminado && (!pose_ready || !vel_ready))   // Bloquear hasta que se reciba la pose actual.
            {
                wait_rate.sleep();  // Esperar para evitar espera activa.
                if (goal_handle->is_canceling())
                {
                    eliminado = true;
                    cancelado = true;
                }
                active_mtx_.lock();
                if (active_goal_ != goal_handle && !eliminado)
                {
                    eliminado = true;
                }
                active_mtx_.unlock();

                pose_callback_mtx_.lock();
                pose_ready = bloquear_callback;
                pose_callback_mtx_.unlock();
                vel_callback_mtx_.lock();
                vel_ready = bloquear_vel;
                vel_callback_mtx_.unlock();
            }

            double t_total = 0.0;
            auto result = std::make_shared<TrayAction::Result>();    // Creamos el objeto de nuestra interfaz que devolveremos al cliente al final del callback.

            if (!eliminado)
            {
                auto feedback_msg = std::make_shared<TrayAction::Feedback>();     // Creamos el objeto de nuestra interfaz que utilizaremos como feedback de la acción

                // Obtenemos los valores del cliente de la acción para utilizarlos en el bucle
                auto goal = goal_handle->get_goal();

                constexpr std::size_t N_EJES_TRAY = 4;

                // Obtener Pose inicial
                pose_callback_mtx_.lock();
                double x0 = last_pose_.pose.position.x;
                double y0 = last_pose_.pose.position.y;
                double z0 = last_pose_.pose.position.z;
                double yaw0 = pose2yaw(last_pose_);
                pose_callback_mtx_.unlock();

                // Obtener Velocidad inicial
                vel_callback_mtx_.lock();
                double vx0 = last_vel_.twist.linear.x;
                double vy0 = last_vel_.twist.linear.y;
                double vz0 = last_vel_.twist.linear.z;
                double vyaw0 = last_vel_.twist.angular.z;
                vel_callback_mtx_.unlock();

                std::vector<std::array<double, 5>> salida_trayectoria(N_EJES_TRAY);

                std::function<void(double, std::vector<std::array<double, 5>> &)> evaluar_trayectoria;

                std::unique_ptr<lib_tray::GenTrayPol3> trayectoria_pol3;
                std::unique_ptr<lib_tray::GenTrayVelTrap> trayectoria_veltrap;
                std::unique_ptr<lib_tray::GenTrayElipse> trayectoria_elipse;

                bool flag_angulo = false;
                bool usar_normalizacion_directa = false;

                try
                {
                    ///////////////////////////////
                    // TIPO 0: POLINOMIO CÚBICO //
                    ///////////////////////////////
                    if (goal->tipo_trayectoria == 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Generando trayectoria tipo 0: pol3");

                        trayectoria_pol3 = std::make_unique<lib_tray::GenTrayPol3>(N_EJES_TRAY);

                        t_total = std::max({static_cast<double>(goal->tx), static_cast<double>(goal->ty), static_cast<double>(goal->tz), static_cast<double>(goal->tyaw)});

                        double xf, yf, yawf;

                        if (goal->absoluto_x)
                        {
                            xf = goal->target_pose.pose.position.x;
                        }
                        else
                        {
                            xf = x0 - goal->target_pose.pose.position.y * sin(yaw0) + goal->target_pose.pose.position.x * cos(yaw0);
                        }

                        if (goal->absoluto_y)
                        {
                            yf = goal->target_pose.pose.position.y;
                        }
                        else
                        {
                            yf = y0 + goal->target_pose.pose.position.y * cos(yaw0) + goal->target_pose.pose.position.x * sin(yaw0);
                        }

                        if (goal->absoluto_yaw)
                        {
                            yawf = pose2yaw(goal->target_pose);
                        }
                        else
                        {
                            yawf = yaw0 + pose2yaw(goal->target_pose);
                            if (yawf > M_PI || yawf < -M_PI)
                            {
                                flag_angulo = true;
                            }
                        }

                        const std::vector<double> posiciones_iniciales = {x0, y0, z0, yaw0};
                        const std::vector<double> posiciones_finales = {xf, yf, goal->target_pose.pose.position.z, yawf};
                        const std::vector<double> velocidades_iniciales = {vx0, vy0, vz0, vyaw0};
                        const std::vector<double> velocidades_finales = {0.0, 0.0, 0.0, 0.0};
                        const std::vector<double> tiempos_finales = {goal->tx, goal->ty, goal->tz, goal->tyaw};
                        const std::vector<bool> finales_absolutas = {true, true, goal->absoluto_z, true};

                        trayectoria_pol3->calcular_trayectoria(posiciones_iniciales, posiciones_finales, velocidades_iniciales, velocidades_finales, tiempos_finales, finales_absolutas);

                        evaluar_trayectoria = [&trayectoria_pol3](double tiempo, std::vector<std::array<double, 5>> & salida)
                        {
                            trayectoria_pol3->evaluar(tiempo, salida);
                        };
                    }

                    //////////////////////////////////////
                    // TIPO 1: VELOCIDAD TRAPEZOIDAL   //
                    //////////////////////////////////////
                    else if (goal->tipo_trayectoria == 1)
                    {
                        RCLCPP_INFO(this->get_logger(), "Generando trayectoria tipo 1: veltrap");

                        trayectoria_veltrap = std::make_unique<lib_tray::GenTrayVelTrap>(N_EJES_TRAY);

                        double xf, yf, zf_abs, yawf;

                        if (goal->absoluto_x)
                        {
                            xf = goal->target_pose.pose.position.x;
                        }
                        else
                        {
                            xf = x0 - goal->target_pose.pose.position.y * sin(yaw0) + goal->target_pose.pose.position.x * cos(yaw0);
                        }

                        if (goal->absoluto_y)
                        {
                            yf = goal->target_pose.pose.position.y;
                        }
                        else
                        {
                            yf = y0 + goal->target_pose.pose.position.y * cos(yaw0) + goal->target_pose.pose.position.x * sin(yaw0);
                        }

                        if (goal->absoluto_z)
                        {
                            zf_abs = goal->target_pose.pose.position.z;
                        }
                        else
                        {
                            zf_abs = z0 + goal->target_pose.pose.position.z;
                        }

                        if (goal->absoluto_yaw)
                        {
                            yawf = pose2yaw(goal->target_pose);
                        }
                        else
                        {
                            yawf = yaw0 + pose2yaw(goal->target_pose);
                            if (yawf > M_PI || yawf < -M_PI)
                            {
                                flag_angulo = true;
                            }
                        }

                        const double vmax_x = proyeccion(v_max_lin, 'X', x0, y0, z0, xf, yf, zf_abs);
                        const double vmax_y = proyeccion(v_max_lin, 'Y', x0, y0, z0, xf, yf, zf_abs);
                        const double vmax_z = proyeccion(v_max_lin, 'Z', x0, y0, z0, xf, yf, zf_abs);

                        const std::vector<double> posiciones_iniciales = {x0, y0, z0, yaw0};
                        const std::vector<double> posiciones_finales = {xf, yf, goal->target_pose.pose.position.z, yawf};
                        const std::vector<double> velocidades_iniciales = {vx0, vy0, vz0, vyaw0};
                        const std::vector<double> velocidades_finales = {0.0, 0.0, 0.0, 0.0};
                        const std::vector<double> velocidades_maximas = {vmax_x, vmax_y, vmax_z, v_max_ang};
                        const std::vector<bool> finales_absolutas = {true, true, goal->absoluto_z, true};

                        trayectoria_veltrap->calcular_trayectoria(posiciones_iniciales, posiciones_finales, velocidades_iniciales, velocidades_finales, velocidades_maximas, t_a, finales_absolutas);

                        const auto & coeficientes = trayectoria_veltrap->get_coeficientes();

                        for (const auto & c : coeficientes)
                        {
                            t_total = std::max(t_total, c.tf);
                        }

                        evaluar_trayectoria = [&trayectoria_veltrap](double tiempo, std::vector<std::array<double, 5>> & salida)
                        {
                            trayectoria_veltrap->evaluar(tiempo, salida);
                        };
                    }

                    ////////////////////
                    // TIPO 2: ELIPSE //
                    ////////////////////
                    else if (goal->tipo_trayectoria == 2)
                    {
                        RCLCPP_INFO(this->get_logger(), "Generando trayectoria tipo 2: elipse");

                        trayectoria_elipse = std::make_unique<lib_tray::GenTrayElipse>(N_EJES_TRAY);

                        double xc, yc, zc;       // centro de la elipse
                        double alpha;            // ángulo del radio x de la elipse
                        double yaw_ref;          // yaw absoluto o offset para mirar al centro

                        const double rx = static_cast<double>(goal->tx);
                        const double ry = static_cast<double>(goal->ty);
                        const double angulo_elipse = static_cast<double>(goal->tz);
                        const double tiempo_vuelta = static_cast<double>(goal->tyaw);

                        if (goal->absoluto_x)
                        {
                            xc = goal->target_pose.pose.position.x;
                        }
                        else
                        {
                            xc = x0 - goal->target_pose.pose.position.y * sin(yaw0) + goal->target_pose.pose.position.x * cos(yaw0);
                        }

                        if (goal->absoluto_y)
                        {
                            yc = goal->target_pose.pose.position.y;
                        }
                        else
                        {
                            yc = y0 + goal->target_pose.pose.position.y * cos(yaw0) + goal->target_pose.pose.position.x * sin(yaw0);
                        }

                        if (goal->absoluto_z)
                        {
                            zc = goal->target_pose.pose.position.z;
                        }
                        else
                        {
                            zc = z0 + goal->target_pose.pose.position.z;
                        }

                        // Si el centro x/y es absoluto, el ángulo de la elipse es respecto a world.
                        // Si el centro x/y es relativo, el ángulo de la elipse es respecto al dron.
                        if (goal->absoluto_x && goal->absoluto_y)
                        {
                            alpha = angulo_elipse;
                        }
                        else
                        {
                            alpha = yaw0 + angulo_elipse;
                        }

                        if (goal->absoluto_yaw)
                        {
                            // Yaw fijo absoluto durante toda la vuelta
                            yaw_ref = pose2yaw(goal->target_pose);
                        }
                        else
                        {
                            // Offset respecto al yaw necesario para mirar al centro.
                            // Si yaw_ref = 0, el dron mira siempre al centro.
                            yaw_ref = pose2yaw(goal->target_pose);
                        }

                        const std::vector<double> posiciones_iniciales = {x0, y0, z0, yaw0};

                        // centro_elipse = {xc, yc, zc, yaw_ref}
                        const std::vector<double> centro_elipse = {xc, yc, zc, yaw_ref};

                        // parametros_elipse = {rx, ry, alpha, tiempo_vuelta}
                        const std::vector<double> parametros_elipse = {rx, ry, alpha, tiempo_vuelta};

                        trayectoria_elipse->calcular_trayectoria(posiciones_iniciales, centro_elipse, parametros_elipse, goal->absoluto_yaw, t_a);

                        t_total = trayectoria_elipse->get_coeficientes().tf;

                        usar_normalizacion_directa = true;

                        evaluar_trayectoria = [&trayectoria_elipse](double tiempo, std::vector<std::array<double, 5>> & salida)
                        {
                            trayectoria_elipse->evaluar(tiempo, salida);
                        };
                    }

                    else
                    {
                        throw std::invalid_argument("tipo_trayectoria no válido");
                    }
                }
                catch (const std::exception & e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error calculando trayectoria: %s", e.what());

                    result->success = false;
                    result->t_total = 0.0f;
                    goal_handle->abort(result);

                    return;
                }

                rclcpp::Rate rate(30.0);     // Delay para verlo más claro en el tópico feedback (junto a rate.sleep())

                auto t0 = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "t=%.6f", t0.seconds());

                while (rclcpp::ok() && !eliminado && t <= t_total)
                {
                    active_mtx_.lock();
                    if (active_goal_ != goal_handle)
                    {
                        eliminado = true;
                    }
                    active_mtx_.unlock();

                    if (!goal_handle->is_canceling())
                    {
                        t = (this->get_clock()->now() - t0).seconds();

                        evaluar_trayectoria(t, salida_trayectoria);

                        // Normalizamos yaw para evitar saltos raros fuera de [-pi, pi].
                        if (flag_angulo)
                        {
                            if (salida_trayectoria[3][0] > M_PI)
                            {
                                salida_trayectoria[3][0] -= 2*M_PI;
                            }
                            else if (salida_trayectoria[3][0] < -M_PI)
                            {
                                salida_trayectoria[3][0] += 2*M_PI;
                            }
                        }

                        if (usar_normalizacion_directa)
                        {
                            salida_trayectoria[3][0] = normalizar_angulo(salida_trayectoria[3][0]);
                        }

                        feedback_msg->t_act = static_cast<float>(t);
                        feedback_msg->x   = array_to_msg(salida_trayectoria[0]);
                        feedback_msg->y   = array_to_msg(salida_trayectoria[1]);
                        feedback_msg->z   = array_to_msg(salida_trayectoria[2]);
                        feedback_msg->yaw = array_to_msg(salida_trayectoria[3]);

                        goal_handle->publish_feedback(feedback_msg);    // Envía periódicamente los valores al tópico /feedback

                        rate.sleep();
                    }
                    else
                    {
                        cancelado = true;
                        eliminado = true;
                    }
                }
            }

            if (eliminado)
            {
                result->success = false;
                result->t_total = 0.0f;
                RCLCPP_INFO(this->get_logger(), "Acción CANCELADA");
                if (cancelado)
                {
                    goal_handle->canceled(result);
                }
            }
            else
            {
                result->success = true;
                result->t_total = static_cast<float>(t_total);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Acción finalizada OK");
            }
        }

        rclcpp_action::Server<TrayAction>::SharedPtr objeto_servicio_accion;     // Creamos el objeto del servicio de la acción
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;

        bool bloquear_callback = false;
        bool bloquear_vel = false;

        geometry_msgs::msg::PoseStamped last_pose_;
        geometry_msgs::msg::TwistStamped last_vel_;

        std::mutex active_mtx_;
        std::mutex pose_callback_mtx_;
        std::mutex vel_callback_mtx_;
        std::shared_ptr<GoalHandleTrayAction> active_goal_;  // goal que consideramos "activo"

        double v_max_lin;
        double v_max_ang;
        double t_a;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Servicio_Accion>();
    rclcpp::spin(objeto_nodo);

    rclcpp::shutdown();
    return 0;
}