// Added custom code for legends, see https://github.com/Profactor/cv-plot/issues/11

#pragma once

#include <CvPlot/drawables/Legend.h>
#include <CvPlot/Internal/util.h>
#include <opencv2/opencv.hpp>

namespace CvPlot
{

  class LegendLabel::Impl
  {
  public:
    cv::Scalar _color = cv::Scalar(0, 0, 0);
    std::string _text{};
    cv::Point _position{};
    const int _fontFace = cv::FONT_HERSHEY_SIMPLEX;
    const double _fontScale = .4;
    const int _fontThickness = 1;

    void render(CvPlot::RenderTarget &renderTarget)
    {
      int baseline;
      cv::Size size = cv::getTextSize(_text, _fontFace, _fontScale, _fontThickness, &baseline);
      auto pos = renderTarget.innerToOuter(renderTarget.project(_position)) + cv::Point2d(size.height * 2, size.height / 2);
      cv::putText(renderTarget.outerMat(), _text, pos, _fontFace, _fontScale, _color, _fontThickness, cv::LINE_AA);
    }
  };

  CVPLOT_DEFINE_FUN
  LegendLabel::~LegendLabel()
  {
  }

  CVPLOT_DEFINE_FUN
  LegendLabel::LegendLabel()
  {
  }

  CVPLOT_DEFINE_FUN
  void LegendLabel::render(RenderTarget &renderTarget)
  {
    impl->render(renderTarget);
  }

  CVPLOT_DEFINE_FUN
  void LegendLabel::setPosition(int const &a_x, int const &a_y)
  {
    impl->_position = cv::Point(a_x, a_y);
  }

  CVPLOT_DEFINE_FUN
  void LegendLabel::setText(std::string const &a_str)
  {
    impl->_text = a_str;
  }

  class Legend::Impl
  {
  public:
    CvPlot::Axes *parentAxes{nullptr};
    int _width = 180;
    int _height = 60;
    int _margin = 10;

    void setParentAxes(CvPlot::Axes *a_parentAxes)
    {
      parentAxes = a_parentAxes;
    }

    void render(CvPlot::RenderTarget &renderTarget)
    {
      std::vector<CvPlot::Series *> seriesVec;
      for (const auto &drawable : parentAxes->drawables())
      {
        auto series = dynamic_cast<CvPlot::Series *>(drawable.get());
        if (series)
        {
          seriesVec.push_back(series);
        }
      }
      CvPlot::Axes axes;
      axes.setMargins(5, _width - 2 * _margin - 60, 5, 5)
          .setXLim({-.2, 1.2})
          .setYLim({-.2, seriesVec.size() - 1 + .2})
          .setYReverse();
      for (uint8_t i = 0; i < seriesVec.size(); i++)
      {
        auto &series = *seriesVec[i];
        axes.create<CvPlot::Series>(std::vector<double>{0, 0.25, 0.5, 0.75, 1}, std::vector<uint8_t>{i, i, i, i, i})
            .setLineType(series.getLineType())
            .setLineWidth(series.getLineWidth())
            .setColor(series.getColor())
            .setMarkerType(series.getMarkerType())
            .setMarkerSize(series.getMarkerSize());
        auto &label = axes.create<LegendLabel>();
        label.setPosition(1, (int)i);
        label.setText(series.getName());
      }
      cv::Rect rect(renderTarget.innerMat().cols - _width - _margin, _margin, _width, _height);
      if (rect.x >= 0 && rect.x + rect.width < renderTarget.innerMat().cols && rect.y >= 0 && rect.y + rect.height < renderTarget.innerMat().rows)
      {
        axes.render(renderTarget.innerMat()(rect));
        cv::rectangle(renderTarget.innerMat(), rect, cv::Scalar::all(0));
      }
    }
  };

  CVPLOT_DEFINE_FUN
  Legend::~Legend()
  {
  }

  CVPLOT_DEFINE_FUN
  Legend::Legend()
  {
  }

  CVPLOT_DEFINE_FUN
  void Legend::setParentAxes(CvPlot::Axes *a_parentAxes)
  {
    impl->setParentAxes(a_parentAxes);
  }

  CVPLOT_DEFINE_FUN
  void Legend::render(RenderTarget &renderTarget)
  {
    impl->render(renderTarget);
  }
}