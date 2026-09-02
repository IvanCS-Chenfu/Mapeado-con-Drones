#include "multidron_gui_lib/render_layer.hpp"

#include <utility>

namespace multidron_gui_lib
{

RenderLayer::RenderLayer(QString name, bool visible)
: name_(std::move(name)), visible_(visible)
{
}

const QString & RenderLayer::Name() const
{
  return name_;
}

bool RenderLayer::IsVisible() const
{
  return visible_;
}

void RenderLayer::SetVisible(bool visible)
{
  visible_ = visible;
}

void RenderLayer::Invalidate()
{
  dirty_ = true;
}

bool RenderLayer::NeedsUpload(const void * data_identity, std::uint64_t style_revision) const
{
  return dirty_ || data_identity != data_identity_ || style_revision != style_revision_;
}

void RenderLayer::MarkUploaded(const void * data_identity, std::uint64_t style_revision)
{
  data_identity_ = data_identity;
  style_revision_ = style_revision;
  dirty_ = false;
}

}  // namespace multidron_gui_lib
