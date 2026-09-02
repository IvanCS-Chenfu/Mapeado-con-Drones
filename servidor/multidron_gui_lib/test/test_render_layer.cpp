#include "multidron_gui_lib/render_layer.hpp"

#include <gtest/gtest.h>

TEST(RenderLayer, TracksVisibilityIdentityAndStyleRevision)
{
  multidron_gui_lib::RenderLayer layer("Sparse");
  int first = 1;
  int second = 2;

  EXPECT_TRUE(layer.IsVisible());
  EXPECT_TRUE(layer.NeedsUpload(&first, 0U));
  layer.MarkUploaded(&first, 0U);
  EXPECT_FALSE(layer.NeedsUpload(&first, 0U));
  EXPECT_TRUE(layer.NeedsUpload(&second, 0U));
  EXPECT_TRUE(layer.NeedsUpload(&first, 1U));

  layer.SetVisible(false);
  EXPECT_FALSE(layer.IsVisible());
  layer.Invalidate();
  EXPECT_TRUE(layer.NeedsUpload(&first, 0U));
}
