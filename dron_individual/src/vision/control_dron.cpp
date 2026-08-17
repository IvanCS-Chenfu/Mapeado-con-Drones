#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "dron_individual/action/tray_action.hpp" 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "std_msgs/msg/u_int8.hpp"


#include <cmath>
#include <chrono>
using namespace std::chrono_literals;


using VarAction = dron_individual::action::TrayAction;
using GoalHandleVarAction = rclcpp_action::ClientGoalHandle<VarAction>;

class Clase_Cliente_Accion : public rclcpp::Node 
{
    public:
        Clase_Cliente_Accion() : rclcpp::Node("control_dron")
        {
            pub_ = this->create_publisher<std_msgs::msg::UInt8>("vision/byte_control", 10);

            sub_control_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("vision/keypoint_cercano",10,
                std::bind(&Clase_Cliente_Accion::callbackKeypoint, this, std::placeholders::_1));

            objeto_cliente_accion = rclcpp_action::create_client<VarAction>(this, "AccionTrayectoria");
            while (!objeto_cliente_accion->wait_for_action_server(1s)) 
            {
                RCLCPP_INFO(get_logger(), "Servicio no disponible");
            }

            vel_media = 0.2;
            umbral_t_max = 10.0;    // No es por esto...
            umbral_frente_pared = 0.1; // 6º aprox
            distancia_pared = 1.2;
            umbral_distancia_muro_izq = 1.5;
            step = 0.75;
            profundidad_maxima = 4.0;
            
            byte_control.data = 0x00;

            objeto_timer = this->create_wall_timer(500ms, std::bind(&Clase_Cliente_Accion::control, this));
        }
    private:

        //////////// CALLBACKS ////////////
        void callbackKeypoint(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            byte_control.data = 0;
            pub_->publish(byte_control);

            x_callback = msg->data[0];
            y_callback = msg->data[1];
            z_callback = msg->data[2];
            w_callback = msg->data[3];

            RCLCPP_INFO(this->get_logger(), "XYZW: %.3f %.3f %.3f %.3f", x_callback, y_callback, z_callback, w_callback);

            callback_enviado = true;
        }

        void on_result(const GoalHandleVarAction::WrappedResult & result)
        {
            posicion_alcanzada = result.result->success;
            RCLCPP_INFO(get_logger(),"Resultado recibido: success=%s, t_total=%.3f",posicion_alcanzada ? "true" : "false",result.result->t_total);
        }

        //////////// FUNCIONES ////////////

        void mover_a(double x, double y, double z, double yaw, bool relativo)   // En esta función debería consultar el mapa y ver si me choco con algo
        {
            posicion_alcanzada = false;

            RCLCPP_INFO(get_logger(), "Enviando objetivo");

            VarAction::Goal goal;

            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = this->get_clock()->now();

            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
            goal.target_pose.pose.position.z = z;

            // Convertir yaw a cuaternión
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = yaw;
            goal.target_pose.pose.orientation.w = 0.0;

            // t_i = i / vel_media
            double max_dist = std::max({std::abs(x), std::abs(y), std::abs(z), std::abs(yaw)});
            double t_max = max_dist/vel_media;

            if (t_max < umbral_t_max)
            {
                t_max = umbral_t_max;
            }

            goal.tx = static_cast<float>(t_max);
            goal.ty = static_cast<float>(t_max);
            goal.tz = static_cast<float>(t_max);
            goal.tyaw = static_cast<float>(t_max);

            goal.absoluto_x = relativo;

            rclcpp_action::Client<VarAction>::SendGoalOptions opts;
            opts.result_callback = std::bind(&Clase_Cliente_Accion::on_result, this, std::placeholders::_1);

            objeto_cliente_accion->async_send_goal(goal, opts);
        }

        //////////// CONTROL ////////////

        bool encontrarse()
        {
            bool terminado = false;

            if (pasos == 0)
            {
                mover_a(0.0,0.0,1.0,0.0,true);

                pasos++;
            }

            if (pasos == 1 && posicion_alcanzada)
            {
                // Decir a "disparidad_maxima_ORB()" que empiece a mirar puntos
                byte_control.data |= 0b00000001;
                pub_->publish(byte_control);

                mover_a(0.0,0.0,0.0,2*M_PI,true);

                pasos++;
            }   

            if (pasos == 2 && posicion_alcanzada)
            {
                // Decir a "disparidad_maxima_ORB()" que empiece a mirar puntos y decir a "avisar_punto_cercano()"
                byte_control.data |= 0b00000010;
                pub_->publish(byte_control);

                mover_a(0.0,0.0,0.0,2*M_PI,true);   // Vuelve a girar, pero es interrumpido por el callback  
                
                pasos++;
            }  

            if (pasos == 3 && callback_enviado)
            {
                callback_enviado = false;   

                mover_a(x_callback-0.5,y_callback,z_callback,0.0,true); 

                terminado = true;

                pasos = 0;
            }
            else if (pasos == 3 && posicion_alcanzada)   // Si el calllback ha sido llamado cuando ha dado la segunda vuelta -> perdido -> otra vuelta
            {
                pasos = 1;
                byte_control.data &= 0b11111100;
                pub_->publish(byte_control);
                RCLCPP_INFO(get_logger(), "no encontrado");
            }

            return terminado;
        }


        bool colocarse_frente_pared()
        {
            bool terminado = false;

            if (pasos == 0 && posicion_alcanzada)
            {
                // Decir a "normales_punto()" que me de la normal principal y el punto más cercano
                byte_control.data |= 0b00000100;
                pub_->publish(byte_control);

                pasos++;
            }

            if (pasos == 1 && callback_enviado)
            {
                callback_enviado = false;

                w_callback = abs(w_callback);
                
                if (180-w_callback < -60)
                {
                    distancia_muro_izq = 0.0; // Con el fin de que vuelva a mirar a su izquierda a ver si se va a chocar
                    RCLCPP_INFO(get_logger(), "Ha Girado");
                }

                w_callback = (180-w_callback)*M_PI/180;

                mover_a(x_callback - distancia_pared * cos(-w_callback),y_callback + distancia_pared * sin(-w_callback), 0.0, w_callback, true); 

                if (abs(w_callback) <= umbral_frente_pared)
                {
                    terminado = true;
                }
                pasos = 0;
            }

            return terminado;
        }


        void obtener_nube()
        {   
            // me coloco perpendicular al plano
            if (!frente_pared)
            {
                RCLCPP_INFO(get_logger(), "frente_pared");
                frente_pared = colocarse_frente_pared();    // En cada paso a arriba (en lo escrito) debería ponerse a false esta variable
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Pasos: %d", pasos);

                // Una vez colocado tomo la foto para la nube de puntos.
                if (pasos == 0 && posicion_alcanzada)
                {
                    // llamo a "vision" para que obtenga una nube
                    byte_control.data |= 0b00001000;
                    pub_->publish(byte_control);

                    pasos++;
                }

                // Obtenida la nube compruebo las distancia de la izquierda.
                if (pasos == 1 && callback_enviado)
                {   

                    callback_enviado = false;

                    // Si hay distancia suficiente a la izquierda y veo como sigo según los flags
                    if (!(distancia_muro_izq < umbral_distancia_muro_izq))
                    {
                        mover_a(0.0,step,0.0,0.0,true);

                        distancia_muro_izq -= step;

                        pasos++;
                        
                        RCLCPP_INFO(get_logger(), "Distancia al muro de la Izquierda: %.3f", distancia_muro_izq);
                    }
                    else
                    {
                        mover_a(0.0,0.0,0.0,M_PI_2,true);

                        pasos = 5;
                    }
                }

                // Una vez movido a la izquierda, miro los flags
                if (pasos == 2 && posicion_alcanzada)
                {

                    // llamo a "vision" para que publique los flags
                    byte_control.data |= 0b00010000;
                    pub_->publish(byte_control);

                    pasos++;
                }

                // Obtengo los flags y prosigo
                if (pasos == 3 && callback_enviado)
                {
                    callback_enviado = false;

                    flag_cambio_Z = (bool)x_callback;
                    flag_giro = (bool)y_callback;

                    RCLCPP_INFO(get_logger(),"Cambio de Plano: %s. Girar Plano: %s",flag_cambio_Z ? "true" : "false",flag_giro ? "true" : "false");
                    
                    // consulto variable (flag_cambio_Z): si "True" -> muevo izquierda -> miro profundidad en centro -> me muevo a ella -> vuelvo arriba
                    if (flag_cambio_Z)
                    {
                        flag_cambio_Z = false;

                        // llamo a "vision" para que obtenga la profundidad de la nueva
                        byte_control.data |= 0b00100000;
                        pub_->publish(byte_control);

                        pasos++;
                    }
                    // consulto variable (flag_giro): si "True -> me muevo izquierda_alante -> giro 90º derecha -> vuelvo arriba
                    else if (flag_giro)
                    {
                        flag_giro = false;

                        mover_a(1.3*distancia_pared,step/2,0.0,-M_PI_2,true); 

                        pasos = 0;
                        frente_pared = false;
                        distancia_muro_izq = 0.0; // Con el fin de que vuelva a mirar a su izquierda a ver si se va a chocar

                        RCLCPP_INFO(get_logger(), "Ha Girado");
                    }
                    else
                    {
                        pasos = 0;
                        frente_pared = false;
                    }
                }

                if (pasos == 4 && callback_enviado)
                {
                    callback_enviado = false;

                    // Si z es demasiado, no es cambio de plano, es giro.
                    if (z_callback > profundidad_maxima)
                    {
                        pasos = 3;
                        flag_giro = true;
                        callback_enviado = true;
                    }
                    else
                    {
                        // Me muevo a la profundidad
                        mover_a(z_callback-distancia_pared,step/2,0.0,0.0,true);
                        
                        pasos = 0;
                        frente_pared = false;
                        distancia_muro_izq = 0.0; // Con el fin de que vuelva a mirar a su izquierda a ver si se va a chocar

                        RCLCPP_INFO(get_logger(), "Ha Cambiado de Plano");
                    }
                }

                if (pasos == 5 && posicion_alcanzada)
                {
                    // llamo a "vision" para que obtenga la profundidad de la nueva
                    byte_control.data |= 0b00100000;
                    pub_->publish(byte_control);

                    pasos++;
                }

                if (pasos == 6 && callback_enviado)
                {
                    callback_enviado = false;
                    
                    // Si la distancia de vd es menor al umbral, no giro (porque ya he girado) y vuelta a empezar
                    // Si me he confundido y la distancia es mayor al umbral, corrijo la distancia y vuelvo a mi posición.
                    if (!(z_callback < umbral_distancia_muro_izq))    // La distancia_muro_izq estaba errónea
                    {
                        distancia_muro_izq = z_callback;

                        mover_a(0.0,0.0,0.0,-M_PI_2,true);
                    }
                    
                    RCLCPP_INFO(get_logger(), "Distancia al muro de la Izquierda (OBTENIDA): %.3f", z_callback);

                    pasos = 0;
                    frente_pared = false;
                }
            }

            
            // me coloco perpendicular al plano y llamo a "vision" para que obtenga una nube y la publique
                // si 2 distancias -> flag_cambio_Z = "True"
                // si 1 distancia y mucho "nan" (muy lejos) -> flag_giro = "True"
                // si 1 distancia nada más -> "ELSE"

            // consulto variable (distancia_muro_izq)
                // si (distancia_muro_izq < umbral) = "False" -> muevo izquerda -> actualizo (distancia_muro_izq) -> vuelvo arriba
                // si (distancia_muro_izq < umbral) = "True" -> giro 180º izquierda -> miro profundidad en centro -> actualizo (distancia_muro_izq) -> giro 90º derecha -> vuelvo arriba

            // consulto variable (flag_cambio_Z)
                // si "True" -> muevo izquierda -> miro profundidad en centro -> me muevo a ella -> vuelvo arriba

            // consulto variable (flag_giro)
                // si "True -> me muevo izquierda_alante -> giro 90º derecha -> vuelvo arriba
        }   
        
        void control()
        {   
            /*
            byte_control.data |= 0b00001000;
            pub_->publish(byte_control);
            */

            /*
            //RCLCPP_INFO(get_logger(),"Encontrado: %s. Frente Pared: %s",encontrado ? "true" : "false",frente_pared ? "true" : "false");

            if (!encontrado)    // Si no sabemos donde estamos, buscamos el punto más cercano
            {
                encontrado = encontrarse();
            }
            
            else */if (posicion_alcanzada)
            {
                obtener_nube();
            }
            
        }

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_;
        rclcpp_action::Client<VarAction>::SharedPtr objeto_cliente_accion;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_control_;
        rclcpp::TimerBase::SharedPtr objeto_timer; 

        double vel_media;
        double umbral_t_max;

        std_msgs::msg::UInt8 byte_control;

        // Variables control
        float x_callback = 0.0;
        float y_callback = 0.0;
        float z_callback = 0.0;
        float w_callback = 0.0;

        int pasos = 0;

        bool callback_enviado = false;
        bool posicion_alcanzada = true;    // HAY QUE PONERLO A FALSE

        bool encontrado = false;
        bool frente_pared = false;

        float distancia_pared;

        float umbral_frente_pared;
        float umbral_distancia_muro_izq;

        float step;

        float distancia_muro_izq = 0.0;

        double profundidad_maxima;

        bool flag_cambio_Z = false;
        bool flag_giro = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Cliente_Accion>(); 

    rclcpp::spin(objeto_nodo); 

    rclcpp::shutdown();
    return 0;
}




// Lo de que se acerque al punto más cercano (mejor solo orientación?)