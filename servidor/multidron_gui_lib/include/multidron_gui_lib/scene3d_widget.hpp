#pragma once

#include "multidron_gui_lib/types.hpp"

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>

#include <memory>
#include <vector>

namespace multidron_gui_lib
{

class Scene3DWidget final : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT

public:
  explicit Scene3DWidget(QWidget * parent = nullptr);
  ~Scene3DWidget() override;

  void SetSnapshot(const GuiSnapshot & snapshot);

  void SetSparseVisible(bool visible);
  void SetKeyframesVisible(bool visible);
  void SetDronesVisible(bool visible);
  void SetFiducialsVisible(bool visible);
  void SetTrajectoriesVisible(bool visible);
  void SetVoxelsVisible(bool visible);

  void SetScoreColorEnabled(bool enabled);
  void SetScoreFilterEnabled(bool enabled);
  void SetScoreThreshold(float threshold);

  QSize minimumSizeHint() const override;

signals:
  void SelectionChanged(const QString & description);

protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;

  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mouseDoubleClickEvent(QMouseEvent * event) override;

private:
  struct Vertex
  {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float r = 1.0F;
    float g = 1.0F;
    float b = 1.0F;
    float a = 1.0F;
  };

  struct GpuLayer
  {
    QOpenGLBuffer buffer{QOpenGLBuffer::VertexBuffer};
    int count = 0;
    unsigned int primitive = 0;
    float point_size = 1.0F;
    bool created = false;
  };

  void CleanupGl();
  void ResetCamera();
  QMatrix4x4 ProjectionMatrix() const;
  QMatrix4x4 ViewMatrix() const;
  QMatrix4x4 MvpMatrix() const;
  QVector3D EyePosition() const;

  void EnsureLayerBuffers();
  void UploadLayer(
    GpuLayer * layer,
    const std::vector<Vertex> & vertices,
    unsigned int primitive,
    float point_size = 1.0F);
  void DrawLayer(GpuLayer & layer, const QMatrix4x4 & mvp);
  void SynchronizeGpuData();

  std::vector<Vertex> BuildGridVertices() const;
  std::vector<Vertex> BuildSparseVertices() const;
  std::vector<Vertex> BuildKeyframeVertices() const;
  std::vector<Vertex> BuildDroneVertices() const;
  std::vector<Vertex> BuildFiducialVertices() const;
  std::vector<Vertex> BuildTrajectoryVertices() const;
  std::vector<Vertex> BuildVoxelWireVertices() const;
  std::vector<Vertex> BuildVoxelFillVertices() const;
  std::vector<Vertex> BuildSelectionVertices() const;

  void PickAt(const QPoint & screen_position);
  bool ProjectToScreen(const QVector3D & world, QPointF * screen, float * depth = nullptr) const;

  static void AppendLine(
    std::vector<Vertex> * vertices,
    const QVector3D & a,
    const QVector3D & b,
    const QColor & color,
    float alpha_multiplier = 1.0F);
  static void AppendBoxEdges(
    std::vector<Vertex> * vertices,
    const QVector3D & center,
    const QVector3D & size,
    const QQuaternion & orientation,
    const QColor & color,
    float alpha_multiplier = 1.0F);
  static void AppendCubeTriangles(
    std::vector<Vertex> * vertices,
    const QVector3D & center,
    float size,
    const QColor & color,
    float alpha);
  static QColor ScoreColor(float score);

  GuiSnapshot snapshot_;

  bool sparse_visible_ = true;
  bool keyframes_visible_ = true;
  bool drones_visible_ = true;
  bool fiducials_visible_ = true;
  bool trajectories_visible_ = true;
  bool voxels_visible_ = false;
  bool score_color_enabled_ = false;
  bool score_filter_enabled_ = false;
  float score_threshold_ = 0.0F;

  float camera_yaw_deg_ = 45.0F;
  float camera_pitch_deg_ = 28.0F;
  float camera_distance_ = 28.0F;
  QVector3D camera_target_{0.0F, 0.0F, 1.0F};

  QPoint last_mouse_position_;
  Qt::MouseButtons pressed_buttons_;
  bool drag_happened_ = false;

  bool selection_valid_ = false;
  QVector3D selection_world_;

  QOpenGLShaderProgram shader_;
  int position_attribute_ = -1;
  int color_attribute_ = -1;
  int mvp_uniform_ = -1;
  int point_size_uniform_ = -1;

  GpuLayer grid_layer_;
  GpuLayer sparse_layer_;
  GpuLayer keyframe_layer_;
  GpuLayer drone_layer_;
  GpuLayer fiducial_layer_;
  GpuLayer trajectory_layer_;
  GpuLayer voxel_wire_layer_;
  GpuLayer voxel_fill_layer_;
  GpuLayer selection_layer_;

  const void * sparse_identity_ = nullptr;
  const void * keyframe_identity_ = nullptr;
  const void * drone_identity_ = nullptr;
  const void * fiducial_identity_ = nullptr;
  const void * trajectory_identity_ = nullptr;
  const void * voxel_identity_ = nullptr;
  bool sparse_style_dirty_ = true;
  bool selection_dirty_ = true;
  bool grid_uploaded_ = false;
};

}  // namespace multidron_gui_lib
