#include "multidron_gui_lib/scene3d_widget.hpp"

#include <QDebug>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QOpenGLShader>
#include <QSurfaceFormat>
#include <QVector4D>
#include <QWheelEvent>
#include <QtGlobal>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>

namespace multidron_gui_lib
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;

QVector3D RotateVector(const QQuaternion & q, const QVector3D & v)
{
  return q.rotatedVector(v);
}

QString TrackingName(std::int8_t state)
{
  switch (state) {
    case -1: return "SYSTEM_NOT_READY";
    case 0: return "NO_IMAGES_YET";
    case 1: return "NOT_INITIALIZED";
    case 2: return "OK";
    case 3: return "RECENTLY_LOST";
    case 4: return "LOST";
    case 5: return "OK_KLT";
    default: return QString("UNKNOWN(%1)").arg(state);
  }
}

QString VoxelStateName(VoxelState state)
{
  switch (state) {
    case VoxelState::Occupied: return "occupied";
    case VoxelState::Free: return "free";
    case VoxelState::Unknown:
    default: return "unknown";
  }
}

QPoint MousePosition(const QMouseEvent * event)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  return event->position().toPoint();
#else
  return event->pos();
#endif
}

}  // namespace

Scene3DWidget::Scene3DWidget(QWidget * parent)
: QOpenGLWidget(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
}

Scene3DWidget::~Scene3DWidget()
{
  CleanupGl();
}

void Scene3DWidget::SetSnapshot(const GuiSnapshot & snapshot)
{
  snapshot_ = snapshot;
  update();
}

void Scene3DWidget::SetSparseVisible(bool visible)
{
  sparse_visible_ = visible;
  update();
}

void Scene3DWidget::SetKeyframesVisible(bool visible)
{
  keyframes_visible_ = visible;
  update();
}

void Scene3DWidget::SetDronesVisible(bool visible)
{
  drones_visible_ = visible;
  update();
}

void Scene3DWidget::SetFiducialsVisible(bool visible)
{
  fiducials_visible_ = visible;
  update();
}

void Scene3DWidget::SetTrajectoriesVisible(bool visible)
{
  trajectories_visible_ = visible;
  update();
}

void Scene3DWidget::SetVoxelsVisible(bool visible)
{
  voxels_visible_ = visible;
  update();
}

void Scene3DWidget::SetScoreColorEnabled(bool enabled)
{
  score_color_enabled_ = enabled;
  sparse_style_dirty_ = true;
  update();
}

void Scene3DWidget::SetScoreFilterEnabled(bool enabled)
{
  score_filter_enabled_ = enabled;
  sparse_style_dirty_ = true;
  update();
}

void Scene3DWidget::SetScoreThreshold(float threshold)
{
  score_threshold_ = std::clamp(threshold, 0.0F, 1.0F);
  sparse_style_dirty_ = true;
  update();
}

QSize Scene3DWidget::minimumSizeHint() const
{
  return QSize(640, 420);
}

void Scene3DWidget::initializeGL()
{
  initializeOpenGLFunctions();
  glEnable(GL_DEPTH_TEST);
#if defined(GL_PROGRAM_POINT_SIZE)
  glEnable(GL_PROGRAM_POINT_SIZE);
#elif defined(GL_VERTEX_PROGRAM_POINT_SIZE)
  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#endif
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  const char * vendor = reinterpret_cast<const char *>(glGetString(GL_VENDOR));
  const char * renderer = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
  const char * version = reinterpret_cast<const char *>(glGetString(GL_VERSION));
  qInfo().noquote() << "[GUI-OPENGL] vendor=" << (vendor ? vendor : "unknown")
                    << " renderer=" << (renderer ? renderer : "unknown")
                    << " version=" << (version ? version : "unknown");

  static const char * kVertexShader = R"(
    #version 120
    attribute vec3 a_position;
    attribute vec4 a_color;
    uniform mat4 u_mvp;
    uniform float u_point_size;
    varying vec4 v_color;
    void main()
    {
      gl_Position = u_mvp * vec4(a_position, 1.0);
      gl_PointSize = u_point_size;
      v_color = a_color;
    }
  )";

  static const char * kFragmentShader = R"(
    #version 120
    varying vec4 v_color;
    void main()
    {
      gl_FragColor = v_color;
    }
  )";

  if (!shader_.addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader) ||
    !shader_.addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader) ||
    !shader_.link())
  {
    qCritical().noquote() << "[GUI-OPENGL-SHADER-ERROR]" << shader_.log();
  }

  position_attribute_ = shader_.attributeLocation("a_position");
  color_attribute_ = shader_.attributeLocation("a_color");
  mvp_uniform_ = shader_.uniformLocation("u_mvp");
  point_size_uniform_ = shader_.uniformLocation("u_point_size");

  EnsureLayerBuffers();
  grid_uploaded_ = false;
  sparse_style_dirty_ = true;
  selection_dirty_ = true;
}

void Scene3DWidget::resizeGL(int width, int height)
{
  glViewport(0, 0, width, height);
}

void Scene3DWidget::paintGL()
{
  glClearColor(0.035F, 0.043F, 0.055F, 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!shader_.isLinked()) {
    return;
  }

  SynchronizeGpuData();
  const QMatrix4x4 mvp = MvpMatrix();

  DrawLayer(grid_layer_, mvp);
  if (voxels_visible_) {
    DrawLayer(voxel_wire_layer_, mvp);
    glDepthMask(GL_FALSE);
    DrawLayer(voxel_fill_layer_, mvp);
    glDepthMask(GL_TRUE);
  }
  if (sparse_visible_) {
    DrawLayer(sparse_layer_, mvp);
  }
  if (keyframes_visible_) {
    DrawLayer(keyframe_layer_, mvp);
  }
  if (fiducials_visible_) {
    DrawLayer(fiducial_layer_, mvp);
  }
  if (trajectories_visible_) {
    DrawLayer(trajectory_layer_, mvp);
  }
  if (drones_visible_) {
    DrawLayer(drone_layer_, mvp);
  }

  glDisable(GL_DEPTH_TEST);
  DrawLayer(selection_layer_, mvp);
  glEnable(GL_DEPTH_TEST);
}

void Scene3DWidget::mousePressEvent(QMouseEvent * event)
{
  last_mouse_position_ = MousePosition(event);
  pressed_buttons_ = event->buttons();
  drag_happened_ = false;
  QOpenGLWidget::mousePressEvent(event);
}

void Scene3DWidget::mouseMoveEvent(QMouseEvent * event)
{
  const QPoint current_position = MousePosition(event);
  const QPoint delta = current_position - last_mouse_position_;
  if (delta.manhattanLength() > 1) {
    drag_happened_ = true;
  }

  if (pressed_buttons_ & Qt::LeftButton) {
    camera_yaw_deg_ -= static_cast<float>(delta.x()) * 0.35F;
    camera_pitch_deg_ = std::clamp(
      camera_pitch_deg_ + static_cast<float>(delta.y()) * 0.30F,
      -85.0F, 85.0F);
    update();
  } else if ((pressed_buttons_ & Qt::RightButton) || (pressed_buttons_ & Qt::MiddleButton)) {
    const QVector3D eye = EyePosition();
    QVector3D forward = (camera_target_ - eye).normalized();
    QVector3D right = QVector3D::crossProduct(forward, QVector3D(0.0F, 0.0F, 1.0F));
    if (right.lengthSquared() < 1e-6F) {
      right = QVector3D(1.0F, 0.0F, 0.0F);
    }
    right.normalize();
    QVector3D up = QVector3D::crossProduct(right, forward).normalized();
    const float scale = camera_distance_ * 0.0015F;
    camera_target_ += (-static_cast<float>(delta.x()) * right +
      static_cast<float>(delta.y()) * up) * scale;
    update();
  }

  last_mouse_position_ = current_position;
  QOpenGLWidget::mouseMoveEvent(event);
}

void Scene3DWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton && !drag_happened_) {
    PickAt(MousePosition(event));
  }
  pressed_buttons_ = event->buttons();
  QOpenGLWidget::mouseReleaseEvent(event);
}

void Scene3DWidget::wheelEvent(QWheelEvent * event)
{
  const float steps = static_cast<float>(event->angleDelta().y()) / 120.0F;
  camera_distance_ *= std::exp(-0.16F * steps);
  camera_distance_ = std::clamp(camera_distance_, 0.6F, 500.0F);
  update();
  event->accept();
}

void Scene3DWidget::mouseDoubleClickEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    ResetCamera();
    update();
  }
  QOpenGLWidget::mouseDoubleClickEvent(event);
}

void Scene3DWidget::CleanupGl()
{
  if (!context()) {
    return;
  }
  makeCurrent();
  for (GpuLayer * layer : {
      &grid_layer_, &sparse_layer_, &keyframe_layer_, &drone_layer_,
      &fiducial_layer_, &trajectory_layer_, &voxel_wire_layer_,
      &voxel_fill_layer_, &selection_layer_})
  {
    if (layer->created) {
      layer->buffer.destroy();
      layer->created = false;
      layer->count = 0;
    }
  }
  shader_.removeAllShaders();
  doneCurrent();
}

void Scene3DWidget::ResetCamera()
{
  camera_yaw_deg_ = 45.0F;
  camera_pitch_deg_ = 28.0F;
  camera_distance_ = 28.0F;
  camera_target_ = QVector3D(0.0F, 0.0F, 1.0F);
}

QMatrix4x4 Scene3DWidget::ProjectionMatrix() const
{
  QMatrix4x4 projection;
  const float aspect = height() > 0 ? static_cast<float>(width()) / height() : 1.0F;
  projection.perspective(45.0F, aspect, 0.05F, 1000.0F);
  return projection;
}

QMatrix4x4 Scene3DWidget::ViewMatrix() const
{
  QMatrix4x4 view;
  view.lookAt(EyePosition(), camera_target_, QVector3D(0.0F, 0.0F, 1.0F));
  return view;
}

QMatrix4x4 Scene3DWidget::MvpMatrix() const
{
  return ProjectionMatrix() * ViewMatrix();
}

QVector3D Scene3DWidget::EyePosition() const
{
  const float yaw = camera_yaw_deg_ * kPi / 180.0F;
  const float pitch = camera_pitch_deg_ * kPi / 180.0F;
  const QVector3D direction(
    std::cos(pitch) * std::cos(yaw),
    std::cos(pitch) * std::sin(yaw),
    std::sin(pitch));
  return camera_target_ + direction * camera_distance_;
}

void Scene3DWidget::EnsureLayerBuffers()
{
  for (GpuLayer * layer : {
      &grid_layer_, &sparse_layer_, &keyframe_layer_, &drone_layer_,
      &fiducial_layer_, &trajectory_layer_, &voxel_wire_layer_,
      &voxel_fill_layer_, &selection_layer_})
  {
    if (!layer->created) {
      layer->created = layer->buffer.create();
      layer->buffer.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    }
  }
}

void Scene3DWidget::UploadLayer(
  GpuLayer * layer,
  const std::vector<Vertex> & vertices,
  unsigned int primitive,
  float point_size)
{
  if (layer == nullptr || !layer->created) {
    return;
  }
  layer->primitive = primitive;
  layer->point_size = point_size;
  layer->count = static_cast<int>(vertices.size());
  layer->buffer.bind();
  layer->buffer.allocate(
    vertices.empty() ? nullptr : vertices.data(),
    static_cast<int>(vertices.size() * sizeof(Vertex)));
  layer->buffer.release();
}

void Scene3DWidget::DrawLayer(GpuLayer & layer, const QMatrix4x4 & mvp)
{
  if (!layer.created || layer.count <= 0) {
    return;
  }
  shader_.bind();
  shader_.setUniformValue(mvp_uniform_, mvp);
  shader_.setUniformValue(point_size_uniform_, layer.point_size);
  layer.buffer.bind();
  shader_.enableAttributeArray(position_attribute_);
  shader_.enableAttributeArray(color_attribute_);
  shader_.setAttributeBuffer(
    position_attribute_, GL_FLOAT, offsetof(Vertex, x), 3, sizeof(Vertex));
  shader_.setAttributeBuffer(
    color_attribute_, GL_FLOAT, offsetof(Vertex, r), 4, sizeof(Vertex));
  glDrawArrays(layer.primitive, 0, layer.count);
  shader_.disableAttributeArray(position_attribute_);
  shader_.disableAttributeArray(color_attribute_);
  layer.buffer.release();
  shader_.release();
}

void Scene3DWidget::SynchronizeGpuData()
{
  EnsureLayerBuffers();
  if (!grid_uploaded_) {
    UploadLayer(&grid_layer_, BuildGridVertices(), GL_LINES);
    grid_uploaded_ = true;
  }

  const void * sparse = snapshot_.sparse_points.get();
  if (sparse != sparse_identity_ || sparse_style_dirty_) {
    UploadLayer(&sparse_layer_, BuildSparseVertices(), GL_POINTS, 3.0F);
    sparse_identity_ = sparse;
    sparse_style_dirty_ = false;
  }

  const void * keyframes = snapshot_.keyframes.get();
  if (keyframes != keyframe_identity_) {
    UploadLayer(&keyframe_layer_, BuildKeyframeVertices(), GL_LINES);
    keyframe_identity_ = keyframes;
  }

  const void * drones = snapshot_.drones.get();
  if (drones != drone_identity_) {
    UploadLayer(&drone_layer_, BuildDroneVertices(), GL_LINES);
    drone_identity_ = drones;
  }

  const void * fiducials = snapshot_.fiducials.get();
  if (fiducials != fiducial_identity_) {
    UploadLayer(&fiducial_layer_, BuildFiducialVertices(), GL_LINES);
    fiducial_identity_ = fiducials;
  }

  const void * trajectories = snapshot_.trajectories.get();
  if (trajectories != trajectory_identity_) {
    UploadLayer(&trajectory_layer_, BuildTrajectoryVertices(), GL_LINES);
    trajectory_identity_ = trajectories;
  }

  const void * voxels = snapshot_.voxels.get();
  if (voxels != voxel_identity_) {
    UploadLayer(&voxel_wire_layer_, BuildVoxelWireVertices(), GL_LINES);
    UploadLayer(&voxel_fill_layer_, BuildVoxelFillVertices(), GL_TRIANGLES);
    voxel_identity_ = voxels;
  }

  if (selection_dirty_) {
    UploadLayer(&selection_layer_, BuildSelectionVertices(), GL_LINES);
    selection_dirty_ = false;
  }
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildGridVertices() const
{
  std::vector<Vertex> vertices;
  constexpr int extent = 25;
  vertices.reserve((extent * 2 + 1) * 4);
  for (int index = -extent; index <= extent; ++index) {
    QColor color = index == 0 ? QColor(100, 115, 135, 180) : QColor(70, 80, 95, 85);
    AppendLine(
      &vertices, QVector3D(static_cast<float>(index), -extent, 0.0F),
      QVector3D(static_cast<float>(index), extent, 0.0F), color);
    AppendLine(
      &vertices, QVector3D(-extent, static_cast<float>(index), 0.0F),
      QVector3D(extent, static_cast<float>(index), 0.0F), color);
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildSparseVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.sparse_points) {
    return vertices;
  }
  vertices.reserve(snapshot_.sparse_points->size());
  for (const auto & point : *snapshot_.sparse_points) {
    if (score_filter_enabled_ && point.score < score_threshold_) {
      continue;
    }
    const QColor color = score_color_enabled_ ? ScoreColor(point.score) : QColor(245, 247, 250);
    const float alpha = static_cast<float>(color.alphaF());
    vertices.push_back(Vertex{
      point.position.x(), point.position.y(), point.position.z(),
      static_cast<float>(color.redF()), static_cast<float>(color.greenF()),
      static_cast<float>(color.blueF()), alpha});
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildKeyframeVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.keyframes) {
    return vertices;
  }
  for (const auto & keyframe : *snapshot_.keyframes) {
    for (std::size_t index = 1; index < keyframe.line_points_world.size(); index += 2) {
      AppendLine(
        &vertices, keyframe.line_points_world[index - 1],
        keyframe.line_points_world[index], keyframe.color);
    }
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildDroneVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.drones) {
    return vertices;
  }
  constexpr float axis = 0.65F;
  for (const auto & item : *snapshot_.drones) {
    const DroneState & drone = item.second;
    if (!drone.has_world_pose) {
      continue;
    }
    const float alpha = drone.lost_or_unavailable ? 0.25F : 1.0F;
    AppendLine(
      &vertices, drone.position,
      drone.position + RotateVector(drone.orientation, QVector3D(axis, 0.0F, 0.0F)),
      QColor(239, 83, 80), alpha);
    AppendLine(
      &vertices, drone.position,
      drone.position + RotateVector(drone.orientation, QVector3D(0.0F, axis, 0.0F)),
      QColor(102, 187, 106), alpha);
    AppendLine(
      &vertices, drone.position,
      drone.position + RotateVector(drone.orientation, QVector3D(0.0F, 0.0F, axis)),
      QColor(66, 165, 245), alpha);
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildFiducialVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.fiducials) {
    return vertices;
  }
  for (const auto & object : *snapshot_.fiducials) {
    AppendBoxEdges(
      &vertices, object.position, object.size_m, object.orientation,
      QColor(255, 183, 77, 230));
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildTrajectoryVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.trajectories) {
    return vertices;
  }
  const std::array<QColor, 6> palette = {
    QColor(38, 198, 218), QColor(171, 71, 188), QColor(255, 202, 40),
    QColor(92, 107, 192), QColor(102, 187, 106), QColor(255, 112, 67)};
  for (const auto & item : *snapshot_.trajectories) {
    const auto & trajectory = item.second;
    const QColor color = palette[trajectory.drone_id % palette.size()];
    for (std::size_t index = 1; index < trajectory.samples_world.size(); ++index) {
      AppendLine(
        &vertices, trajectory.samples_world[index - 1],
        trajectory.samples_world[index], color);
    }
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildVoxelWireVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.voxels) {
    return vertices;
  }
  for (const auto & voxel : *snapshot_.voxels) {
    const float size = std::max(voxel.size_m, 0.001F);
    const QColor color = voxel.state == VoxelState::Occupied ?
      QColor(210, 220, 230, 115) : QColor(185, 198, 210, 45);
    AppendBoxEdges(
      &vertices, voxel.center_world, QVector3D(size, size, size), QQuaternion(), color);
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildVoxelFillVertices() const
{
  std::vector<Vertex> vertices;
  if (!snapshot_.voxels) {
    return vertices;
  }
  for (const auto & voxel : *snapshot_.voxels) {
    if (voxel.state != VoxelState::Occupied) {
      // Free/unknown: relleno completamente transparente por contrato visual F7.
      continue;
    }
    const float alpha = 0.15F + 0.75F * std::clamp(voxel.score, 0.0F, 1.0F);
    AppendCubeTriangles(
      &vertices, voxel.center_world, std::max(voxel.size_m, 0.001F),
      QColor(205, 215, 225), alpha);
  }
  return vertices;
}

std::vector<Scene3DWidget::Vertex> Scene3DWidget::BuildSelectionVertices() const
{
  std::vector<Vertex> vertices;
  if (!selection_valid_) {
    return vertices;
  }
  constexpr float half = 0.22F;
  const QColor color(255, 235, 59, 255);
  AppendLine(
    &vertices, selection_world_ - QVector3D(half, 0.0F, 0.0F),
    selection_world_ + QVector3D(half, 0.0F, 0.0F), color);
  AppendLine(
    &vertices, selection_world_ - QVector3D(0.0F, half, 0.0F),
    selection_world_ + QVector3D(0.0F, half, 0.0F), color);
  AppendLine(
    &vertices, selection_world_ - QVector3D(0.0F, 0.0F, half),
    selection_world_ + QVector3D(0.0F, 0.0F, half), color);
  return vertices;
}

void Scene3DWidget::PickAt(const QPoint & screen_position)
{
  constexpr double tolerance_px = 12.0;
  double best_distance = tolerance_px;
  float best_depth = std::numeric_limits<float>::infinity();
  QString description;
  QVector3D best_world;

  auto consider = [&](const QVector3D & world, const QString & text) {
      QPointF projected;
      float depth = 0.0F;
      if (!ProjectToScreen(world, &projected, &depth)) {
        return;
      }
      const double dx = projected.x() - screen_position.x();
      const double dy = projected.y() - screen_position.y();
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < best_distance ||
        (std::abs(distance - best_distance) < 0.5 && depth < best_depth))
      {
        best_distance = distance;
        best_depth = depth;
        best_world = world;
        description = text;
      }
    };

  if (drones_visible_ && snapshot_.drones) {
    for (const auto & item : *snapshot_.drones) {
      const auto & drone = item.second;
      if (!drone.has_world_pose) {
        continue;
      }
      consider(
        drone.position,
        QString("DRONE\nid: %1\ntracking: %2\nmap_epoch: %3\npose_revision: %4\n"
                "x: %5\ny: %6\nz: %7\nyaw: %8 rad\nvisual: %9")
        .arg(drone.drone_id)
        .arg(TrackingName(drone.tracking_state))
        .arg(static_cast<qulonglong>(drone.map_epoch))
        .arg(static_cast<qulonglong>(drone.pose_revision))
        .arg(drone.position.x(), 0, 'f', 3)
        .arg(drone.position.y(), 0, 'f', 3)
        .arg(drone.position.z(), 0, 'f', 3)
        .arg(drone.yaw_rad, 0, 'f', 4)
        .arg(drone.lost_or_unavailable ? "PERDIDO/NO DISPONIBLE" : "OK"));
    }
  }

  if (keyframes_visible_ && snapshot_.keyframes) {
    for (const auto & keyframe : *snapshot_.keyframes) {
      consider(
        keyframe.position,
        QString("KEYFRAME MARKER\nmarker_id: %1\nnamespace: %2\n"
                "x: %3\ny: %4\nz: %5")
        .arg(keyframe.marker_id)
        .arg(QString::fromStdString(keyframe.marker_namespace))
        .arg(keyframe.position.x(), 0, 'f', 3)
        .arg(keyframe.position.y(), 0, 'f', 3)
        .arg(keyframe.position.z(), 0, 'f', 3));
    }
  }

  if (fiducials_visible_ && snapshot_.fiducials) {
    for (const auto & fiducial : *snapshot_.fiducials) {
      QString tags;
      for (std::size_t index = 0; index < fiducial.tag_ids.size(); ++index) {
        if (index != 0U) {
          tags += ", ";
        }
        tags += QString::number(fiducial.tag_ids[index]);
      }
      consider(
        fiducial.position,
        QString("FIDUCIAL OBJECT\nobject_id: %1\nshape: %2\ntags: [%3]\n"
                "x: %4\ny: %5\nz: %6")
        .arg(fiducial.object_id)
        .arg(QString::fromStdString(fiducial.shape))
        .arg(tags)
        .arg(fiducial.position.x(), 0, 'f', 3)
        .arg(fiducial.position.y(), 0, 'f', 3)
        .arg(fiducial.position.z(), 0, 'f', 3));
    }
  }

  if (trajectories_visible_ && snapshot_.trajectories) {
    for (const auto & item : *snapshot_.trajectories) {
      const auto & trajectory = item.second;
      for (const auto & sample : trajectory.samples_world) {
        consider(
          sample,
          QString("TRAJECTORY\ndrone_id: %1\ntask_id: %2\ntrajectory_id: %3\n"
                  "plan_revision: %4\nmap_revision: %5\nalignment_revision: %6")
          .arg(trajectory.drone_id)
          .arg(QString::fromStdString(trajectory.task_id))
          .arg(QString::fromStdString(trajectory.trajectory_id))
          .arg(static_cast<qulonglong>(trajectory.plan_revision))
          .arg(static_cast<qulonglong>(trajectory.map_revision))
          .arg(static_cast<qulonglong>(trajectory.alignment_revision)));
      }
    }
  }

  if (voxels_visible_ && snapshot_.voxels) {
    for (const auto & voxel : *snapshot_.voxels) {
      consider(
        voxel.center_world,
        QString("VOXEL\nindex: (%1, %2, %3)\nstate: %4\nscore: %5\nsize_m: %6")
        .arg(static_cast<qlonglong>(voxel.ix))
        .arg(static_cast<qlonglong>(voxel.iy))
        .arg(static_cast<qlonglong>(voxel.iz))
        .arg(VoxelStateName(voxel.state))
        .arg(voxel.score, 0, 'f', 3)
        .arg(voxel.size_m, 0, 'f', 3));
    }
  }

  if (sparse_visible_ && snapshot_.sparse_points) {
    for (const auto & point : *snapshot_.sparse_points) {
      if (score_filter_enabled_ && point.score < score_threshold_) {
        continue;
      }
      consider(
        point.position,
        QString("MAP POINT\nsource_index: %1\ndrone_id: %2\nmap_epoch: %3\n"
                "score: %4\nx: %5\ny: %6\nz: %7")
        .arg(static_cast<qulonglong>(point.source_index))
        .arg(point.drone_id)
        .arg(static_cast<qulonglong>(point.map_epoch))
        .arg(point.score, 0, 'f', 4)
        .arg(point.position.x(), 0, 'f', 3)
        .arg(point.position.y(), 0, 'f', 3)
        .arg(point.position.z(), 0, 'f', 3));
    }
  }

  if (description.isEmpty()) {
    selection_valid_ = false;
    selection_dirty_ = true;
    emit SelectionChanged("Sin selección");
  } else {
    selection_valid_ = true;
    selection_world_ = best_world;
    selection_dirty_ = true;
    emit SelectionChanged(description);
  }
  update();
}

bool Scene3DWidget::ProjectToScreen(
  const QVector3D & world,
  QPointF * screen,
  float * depth) const
{
  if (screen == nullptr || width() <= 0 || height() <= 0) {
    return false;
  }
  const QVector4D clip = MvpMatrix() * QVector4D(world, 1.0F);
  if (clip.w() <= 1e-6F) {
    return false;
  }
  const QVector3D ndc = clip.toVector3D() / clip.w();
  if (ndc.z() < -1.0F || ndc.z() > 1.0F) {
    return false;
  }
  screen->setX((ndc.x() * 0.5F + 0.5F) * width());
  screen->setY((1.0F - (ndc.y() * 0.5F + 0.5F)) * height());
  if (depth != nullptr) {
    *depth = ndc.z();
  }
  return true;
}

void Scene3DWidget::AppendLine(
  std::vector<Vertex> * vertices,
  const QVector3D & a,
  const QVector3D & b,
  const QColor & color,
  float alpha_multiplier)
{
  if (vertices == nullptr) {
    return;
  }
  const float alpha = std::clamp(
    static_cast<float>(color.alphaF()) * alpha_multiplier, 0.0F, 1.0F);
  const Vertex va{
    a.x(), a.y(), a.z(), static_cast<float>(color.redF()),
    static_cast<float>(color.greenF()), static_cast<float>(color.blueF()), alpha};
  const Vertex vb{
    b.x(), b.y(), b.z(), static_cast<float>(color.redF()),
    static_cast<float>(color.greenF()), static_cast<float>(color.blueF()), alpha};
  vertices->push_back(va);
  vertices->push_back(vb);
}

void Scene3DWidget::AppendBoxEdges(
  std::vector<Vertex> * vertices,
  const QVector3D & center,
  const QVector3D & size,
  const QQuaternion & orientation,
  const QColor & color,
  float alpha_multiplier)
{
  if (vertices == nullptr) {
    return;
  }
  const QVector3D half = size * 0.5F;
  std::array<QVector3D, 8> corners = {
    QVector3D(-half.x(), -half.y(), -half.z()),
    QVector3D( half.x(), -half.y(), -half.z()),
    QVector3D( half.x(),  half.y(), -half.z()),
    QVector3D(-half.x(),  half.y(), -half.z()),
    QVector3D(-half.x(), -half.y(),  half.z()),
    QVector3D( half.x(), -half.y(),  half.z()),
    QVector3D( half.x(),  half.y(),  half.z()),
    QVector3D(-half.x(),  half.y(),  half.z())};
  for (auto & corner : corners) {
    corner = center + orientation.rotatedVector(corner);
  }
  static constexpr std::array<std::array<int, 2>, 12> edges = {{
    {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
    {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
    {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
  for (const auto & edge : edges) {
    AppendLine(vertices, corners[edge[0]], corners[edge[1]], color, alpha_multiplier);
  }
}

void Scene3DWidget::AppendCubeTriangles(
  std::vector<Vertex> * vertices,
  const QVector3D & center,
  float size,
  const QColor & color,
  float alpha)
{
  if (vertices == nullptr) {
    return;
  }
  const float h = size * 0.5F;
  const std::array<QVector3D, 8> p = {
    center + QVector3D(-h, -h, -h), center + QVector3D(h, -h, -h),
    center + QVector3D(h, h, -h), center + QVector3D(-h, h, -h),
    center + QVector3D(-h, -h, h), center + QVector3D(h, -h, h),
    center + QVector3D(h, h, h), center + QVector3D(-h, h, h)};
  static constexpr std::array<std::array<int, 3>, 12> triangles = {{
    {{0, 2, 1}}, {{0, 3, 2}}, {{4, 5, 6}}, {{4, 6, 7}},
    {{0, 1, 5}}, {{0, 5, 4}}, {{1, 2, 6}}, {{1, 6, 5}},
    {{2, 3, 7}}, {{2, 7, 6}}, {{3, 0, 4}}, {{3, 4, 7}}}};
  const Vertex base{
    0.0F, 0.0F, 0.0F, static_cast<float>(color.redF()),
    static_cast<float>(color.greenF()), static_cast<float>(color.blueF()),
    std::clamp(alpha, 0.0F, 1.0F)};
  for (const auto & triangle : triangles) {
    for (const int index : triangle) {
      Vertex vertex = base;
      vertex.x = p[index].x();
      vertex.y = p[index].y();
      vertex.z = p[index].z();
      vertices->push_back(vertex);
    }
  }
}

QColor Scene3DWidget::ScoreColor(float score)
{
  const float normalized = std::clamp(score, 0.0F, 1.0F);
  if (normalized <= 0.5F) {
    return QColor::fromRgbF(1.0, normalized * 2.0, 0.0, 1.0);
  }
  return QColor::fromRgbF(2.0 * (1.0 - normalized), 1.0, 0.0, 1.0);
}

}  // namespace multidron_gui_lib
