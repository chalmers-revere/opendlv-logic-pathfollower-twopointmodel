#pragma once

#include <CvPlot/libdef.h>
#include <opencv2/core.hpp>
#include <CvPlot/Internal/Pimpl.h>
#include <CvPlot/Internal/no_warning.h>
#include <CvPlot/core/Drawable.h>

namespace CvPlot
{

  class CVPLOT_LIBRARY_INTERFACE LegendLabel : public Drawable
  {
  public:
    LegendLabel();
    ~LegendLabel();
    void render(RenderTarget &renderTarget) override;
    void setPosition(int const &a_x, int const &a_y);
    void setText(std::string const &a_str);

  private:
    class Impl;
    CVPLOT_NO_WARNING_DLL_INTERFACE(Internal::Pimpl<Impl>, impl);
  };

  class CVPLOT_LIBRARY_INTERFACE Legend : public Drawable
  {
  public:
    Legend();
    ~Legend();
    void render(RenderTarget &renderTarget) override;
    void setParentAxes(CvPlot::Axes *a_parentAxes);

  private:
    class Impl;
    CVPLOT_NO_WARNING_DLL_INTERFACE(Internal::Pimpl<Impl>, impl);
  };

}
#ifdef CVPLOT_HEADER_ONLY
#include <CvPlot/imp/Legend.ipp>
#endif