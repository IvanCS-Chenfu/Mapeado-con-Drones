#pragma once

#include <QString>

#include <cstdint>

namespace multidron_gui_lib
{

/// Estado común de una capa renderizable. La geometría y el VBO permanecen en
/// Scene3DWidget; esta frontera decide cuándo una capa necesita resincronizarse.
class RenderLayer
{
public:
  explicit RenderLayer(QString name, bool visible = true);

  const QString & Name() const;
  bool IsVisible() const;
  void SetVisible(bool visible);

  void Invalidate();
  bool NeedsUpload(const void * data_identity, std::uint64_t style_revision = 0) const;
  void MarkUploaded(const void * data_identity, std::uint64_t style_revision = 0);

private:
  QString name_;
  bool visible_ = true;
  bool dirty_ = true;
  const void * data_identity_ = nullptr;
  std::uint64_t style_revision_ = 0;
};

}  // namespace multidron_gui_lib
