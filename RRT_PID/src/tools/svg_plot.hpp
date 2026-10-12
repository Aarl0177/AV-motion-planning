// svg_plot.hpp -- a small, dependency-free SVG plotting helper.
//
// Enough of matplotlib's job for this package: line plots, circles, markers,
// horizontal reference lines, text labels, stacked panels, equal aspect,
// "nice" ticks, a legend, and hover tooltips (SVG <title>) in browsers.
//
//   svgplot::Figure fig(1000, 600);
//   auto & ax = fig.addAxes();
//   ax.title = "Speed";  ax.ylabel = "v [m/s]";
//   ax.addLine(t, v, {svgplot::kBlue}, "v");
//   fig.save("speed.svg");
#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace svgplot
{

// Colours: the first two categorical slots of a colour-blind-checked palette,
// plus neutral "ink" for text, reference lines and chart chrome.
inline const std::string kBlue = "#2a78d6";
inline const std::string kOrange = "#eb6834";
inline const std::string kSurface = "#fcfcfb";
inline const std::string kInk = "#0b0b0b";
inline const std::string kInkSecondary = "#52514e";
inline const std::string kInkMuted = "#898781";
inline const std::string kGrid = "#e1e0d9";
inline const std::string kAxis = "#c3c2b7";

struct Style
{
  std::string color{kBlue};
  double width{2.0};
  std::string dash{};       // SVG stroke-dasharray, e.g. "6 4"; empty = solid
  double opacity{1.0};
};

struct Line
{
  std::vector<double> x, y;
  Style style;
  std::string label;        // legend entry ("" = not in legend)
  std::string tooltip;      // hover text
};

struct Circle
{
  double cx, cy, r;         // in data units
  Style style;
  std::string label;
};

struct Marker
{
  double x, y;
  std::string color{kBlue};
  char shape{'o'};          // 'o' filled circle, 'x' cross
  std::string label;
  std::string tooltip;
};

struct HLine
{
  double y;
  Style style;
  std::string text;         // direct label drawn at the right end
  bool text_below{false};   // put the label under the line instead of above it
};

struct Text
{
  double x, y;              // data coordinates
  std::string text;
  std::string anchor{"start"};
  double dx{0}, dy{0};      // pixel offsets
  std::string color{kInkSecondary};
};

inline std::string escape(const std::string & s)
{
  std::string o;
  for (char c : s) {
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      default: o += c;
    }
  }
  return o;
}

inline std::string fmt(double v, int decimals)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(decimals);
  os << v;
  std::string s = os.str();
  if (s.rfind("-0", 0) == 0 && std::stod(s) == 0.0) {
    s = s.substr(1);  // no "-0.0"
  }
  return s;
}

// 1, 2, 5 x 10^k step giving roughly `target` intervals.
inline double niceStep(double range, int target = 6)
{
  if (range <= 0) {
    return 1.0;
  }
  const double raw = range / target;
  const double mag = std::pow(10.0, std::floor(std::log10(raw)));
  const double f = raw / mag;
  const double nf = f < 1.5 ? 1.0 : (f < 3.0 ? 2.0 : (f < 7.0 ? 5.0 : 10.0));
  return nf * mag;
}

class Axes
{
public:
  std::string title, xlabel, ylabel;
  bool equal_aspect{false};
  std::optional<double> xmin, xmax, ymin, ymax;  // fixed limits (optional)
  bool legend_bottom{false};                      // legend in the lower-right corner

  void addLine(
    const std::vector<double> & x, const std::vector<double> & y, const Style & st,
    const std::string & label = "", const std::string & tooltip = "")
  {
    lines_.push_back({x, y, st, label, tooltip.empty() ? label : tooltip});
  }
  void addCircle(double cx, double cy, double r, const Style & st, const std::string & label = "")
  {
    circles_.push_back({cx, cy, r, st, label});
  }
  void addMarker(const Marker & m) {markers_.push_back(m);}
  void addHLine(
    double y, const Style & st, const std::string & text = "", bool text_below = false)
  {
    hlines_.push_back({y, st, text, text_below});
  }
  void addText(const Text & t) {texts_.push_back(t);}

  // Render into the rectangle (x0, y0, w, h) of the SVG document.
  void render(std::ostringstream & os, double x0, double y0, double w, double h, int id) const
  {
    const double ml = 66, mr = 18, mt = title.empty() ? 14 : 34, mb = xlabel.empty() ? 30 : 48;
    const double px = x0 + ml, py = y0 + mt, pw = w - ml - mr, ph = h - mt - mb;

    // ---- data range ----
    double lo_x = std::numeric_limits<double>::max(), hi_x = -lo_x;
    double lo_y = lo_x, hi_y = -lo_x;
    auto grow = [&](double x, double y) {
        if (std::isfinite(x)) {lo_x = std::min(lo_x, x); hi_x = std::max(hi_x, x);}
        if (std::isfinite(y)) {lo_y = std::min(lo_y, y); hi_y = std::max(hi_y, y);}
      };
    for (const auto & l : lines_) {
      for (size_t i = 0; i < l.x.size() && i < l.y.size(); ++i) {grow(l.x[i], l.y[i]);}
    }
    for (const auto & c : circles_) {grow(c.cx - c.r, c.cy - c.r); grow(c.cx + c.r, c.cy + c.r);}
    for (const auto & m : markers_) {grow(m.x, m.y);}
    for (const auto & hl : hlines_) {grow(std::numeric_limits<double>::quiet_NaN(), hl.y);}
    if (lo_x > hi_x) {lo_x = 0; hi_x = 1;}
    if (lo_y > hi_y) {lo_y = 0; hi_y = 1;}
    if (hi_x - lo_x < 1e-9) {lo_x -= 0.5; hi_x += 0.5;}
    if (hi_y - lo_y < 1e-9) {lo_y -= 0.5; hi_y += 0.5;}
    const double pad_x = 0.03 * (hi_x - lo_x), pad_y = 0.06 * (hi_y - lo_y);
    double x_lo = xmin.value_or(lo_x - pad_x), x_hi = xmax.value_or(hi_x + pad_x);
    double y_lo = ymin.value_or(lo_y - pad_y), y_hi = ymax.value_or(hi_y + pad_y);

    if (equal_aspect) {  // same metres-per-pixel on both axes
      const double sx = pw / (x_hi - x_lo), sy = ph / (y_hi - y_lo);
      if (sx < sy) {
        const double c = 0.5 * (y_lo + y_hi), half = 0.5 * ph / sx;
        y_lo = c - half; y_hi = c + half;
      } else {
        const double c = 0.5 * (x_lo + x_hi), half = 0.5 * pw / sy;
        x_lo = c - half; x_hi = c + half;
      }
    }
    auto X = [&](double x) {return px + (x - x_lo) / (x_hi - x_lo) * pw;};
    auto Y = [&](double y) {return py + ph - (y - y_lo) / (y_hi - y_lo) * ph;};
    const double ppu_x = pw / (x_hi - x_lo);

    // ---- grid, ticks, axes ----
    os << "<g font-family=\"system-ui, -apple-system, 'Segoe UI', sans-serif\">\n";
    auto ticks = [](double lo, double hi, int target) {
        const double step = niceStep(hi - lo, target);
        std::vector<double> t;
        for (double v = std::ceil(lo / step) * step; v <= hi + 1e-9 * step; v += step) {
          t.push_back(std::fabs(v) < 1e-12 ? 0.0 : v);
        }
        const int dec = step >= 1.0 ? 0 : static_cast<int>(std::ceil(-std::log10(step) - 1e-9));
        return std::make_pair(t, dec);
      };
    const auto [xt, xdec] = ticks(x_lo, x_hi, std::max(4, static_cast<int>(pw / 90)));
    const auto [yt, ydec] = ticks(y_lo, y_hi, std::max(3, static_cast<int>(ph / 55)));
    for (double v : xt) {
      os << "<line x1=\"" << X(v) << "\" y1=\"" << py << "\" x2=\"" << X(v) << "\" y2=\""
         << py + ph << "\" stroke=\"" << kGrid << "\" stroke-width=\"1\"/>\n"
         << "<text x=\"" << X(v) << "\" y=\"" << py + ph + 17 << "\" font-size=\"12\" fill=\""
         << kInkMuted << "\" text-anchor=\"middle\">" << fmt(v, xdec) << "</text>\n";
    }
    for (double v : yt) {
      os << "<line x1=\"" << px << "\" y1=\"" << Y(v) << "\" x2=\"" << px + pw << "\" y2=\""
         << Y(v) << "\" stroke=\"" << kGrid << "\" stroke-width=\"1\"/>\n"
         << "<text x=\"" << px - 8 << "\" y=\"" << Y(v) + 4 << "\" font-size=\"12\" fill=\""
         << kInkMuted << "\" text-anchor=\"end\">" << fmt(v, ydec) << "</text>\n";
    }
    os << "<line x1=\"" << px << "\" y1=\"" << py + ph << "\" x2=\"" << px + pw << "\" y2=\""
       << py + ph << "\" stroke=\"" << kAxis << "\" stroke-width=\"1\"/>\n";
    if (!title.empty()) {
      os << "<text x=\"" << px << "\" y=\"" << y0 + 22 << "\" font-size=\"15\" font-weight=\"600\""
         << " fill=\"" << kInk << "\">" << escape(title) << "</text>\n";
    }
    if (!xlabel.empty()) {
      os << "<text x=\"" << px + pw / 2 << "\" y=\"" << py + ph + 38 << "\" font-size=\"13\""
         << " fill=\"" << kInkSecondary << "\" text-anchor=\"middle\">" << escape(xlabel)
         << "</text>\n";
    }
    if (!ylabel.empty()) {
      const double yx = x0 + 16, yy = py + ph / 2;
      os << "<text x=\"" << yx << "\" y=\"" << yy << "\" font-size=\"13\" fill=\""
         << kInkSecondary << "\" text-anchor=\"middle\" transform=\"rotate(-90 " << yx << " "
         << yy << ")\">" << escape(ylabel) << "</text>\n";
    }

    // ---- data, clipped to the plot area ----
    os << "<clipPath id=\"clip" << id << "\"><rect x=\"" << px << "\" y=\"" << py
       << "\" width=\"" << pw << "\" height=\"" << ph << "\"/></clipPath>\n"
       << "<g clip-path=\"url(#clip" << id << ")\">\n";
    auto stroke = [](const Style & s) {
        std::ostringstream st;
        st << "stroke=\"" << s.color << "\" stroke-width=\"" << s.width
           << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\"";
        if (!s.dash.empty()) {st << " stroke-dasharray=\"" << s.dash << "\"";}
        if (s.opacity < 1.0) {st << " stroke-opacity=\"" << s.opacity << "\"";}
        return st.str();
      };
    for (const auto & hl : hlines_) {
      os << "<line x1=\"" << px << "\" y1=\"" << Y(hl.y) << "\" x2=\"" << px + pw << "\" y2=\""
         << Y(hl.y) << "\" " << stroke(hl.style) << "/>\n";
    }
    for (const auto & c : circles_) {
      os << "<circle cx=\"" << X(c.cx) << "\" cy=\"" << Y(c.cy) << "\" r=\"" << c.r * ppu_x
         << "\" fill=\"none\" " << stroke(c.style) << "/>\n";
    }
    for (const auto & l : lines_) {
      os << "<path fill=\"none\" " << stroke(l.style) << " d=\"";
      bool pen_down = false;
      for (size_t i = 0; i < l.x.size() && i < l.y.size(); ++i) {
        if (!std::isfinite(l.x[i]) || !std::isfinite(l.y[i])) {pen_down = false; continue;}
        os << (pen_down ? " L" : " M") << X(l.x[i]) << " " << Y(l.y[i]);
        pen_down = true;
      }
      os << "\">";
      if (!l.tooltip.empty()) {os << "<title>" << escape(l.tooltip) << "</title>";}
      os << "</path>\n";
    }
    os << "</g>\n";

    // markers and labels on top (not clipped, so end points stay visible)
    for (const auto & m : markers_) {
      os << "<g>";
      if (!m.tooltip.empty()) {os << "<title>" << escape(m.tooltip) << "</title>";}
      drawMarker(os, X(m.x), Y(m.y), m.color, m.shape);
      os << "</g>\n";
    }
    for (const auto & hl : hlines_) {
      if (!hl.text.empty()) {
        os << "<text x=\"" << px + pw - 4 << "\" y=\"" << Y(hl.y) + (hl.text_below ? 15 : -5)
           << "\" font-size=\"12\""
           << " fill=\"" << kInkSecondary << "\" text-anchor=\"end\">" << escape(hl.text)
           << "</text>\n";
      }
    }
    for (const auto & t : texts_) {
      os << "<text x=\"" << X(t.x) + t.dx << "\" y=\"" << Y(t.y) + t.dy << "\" font-size=\"12\""
         << " fill=\"" << t.color << "\" text-anchor=\"" << t.anchor << "\" paint-order=\"stroke\""
         << " stroke=\"" << kSurface << "\" stroke-width=\"3\">" << escape(t.text) << "</text>\n";
    }

    renderLegend(os, px, py, pw, ph);
    os << "</g>\n";
  }

private:
  static void drawMarker(
    std::ostringstream & os, double x, double y, const std::string & color, char shape)
  {
    if (shape == 'x') {
      const double s = 5.5;
      for (const char * ring : {"5", "2.5"}) {  // surface ring under the cross
        const bool under = std::string(ring) == "5";
        os << "<path d=\"M" << x - s << " " << y - s << " L" << x + s << " " << y + s << " M"
           << x - s << " " << y + s << " L" << x + s << " " << y - s << "\" stroke=\""
           << (under ? kSurface : color) << "\" stroke-width=\"" << ring
           << "\" stroke-linecap=\"round\"/>";
      }
    } else {
      os << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"5\" fill=\"" << color
         << "\" stroke=\"" << kSurface << "\" stroke-width=\"2\"/>";
    }
  }

  void renderLegend(std::ostringstream & os, double px, double py, double pw, double ph) const
  {
    struct Entry {std::string label; int kind; Style st; char shape;};  // 0 line, 1 circle, 2 marker
    std::vector<Entry> e;
    for (const auto & l : lines_) {if (!l.label.empty()) {e.push_back({l.label, 0, l.style, 'o'});}}
    for (const auto & c : circles_) {if (!c.label.empty()) {e.push_back({c.label, 1, c.style, 'o'});}}
    for (const auto & m : markers_) {
      if (!m.label.empty()) {e.push_back({m.label, 2, Style{m.color}, m.shape});}
    }
    if (e.size() < 2) {
      return;  // a single series is named by the title, no legend box
    }
    size_t longest = 0;
    for (const auto & en : e) {longest = std::max(longest, en.label.size());}
    const double row = 19, w = 44 + 6.6 * longest, h = 10 + row * e.size();
    const double lx = px + pw - w - 8, ly = legend_bottom ? py + ph - h - 8 : py + 8;
    os << "<rect x=\"" << lx << "\" y=\"" << ly << "\" width=\"" << w << "\" height=\"" << h
       << "\" rx=\"6\" fill=\"" << kSurface << "\" fill-opacity=\"0.94\""
       << " stroke=\"rgba(11,11,11,0.10)\"/>\n";
    for (size_t i = 0; i < e.size(); ++i) {
      const double cy = ly + 5 + row * (i + 0.5), sx = lx + 10;
      const auto & st = e[i].st;
      if (e[i].kind == 0) {
        os << "<line x1=\"" << sx << "\" y1=\"" << cy << "\" x2=\"" << sx + 22 << "\" y2=\"" << cy
           << "\" stroke=\"" << st.color << "\" stroke-width=\"" << std::max(2.0, st.width)
           << "\" stroke-linecap=\"round\""
           << (st.dash.empty() ? "" : " stroke-dasharray=\"" + st.dash + "\"")
           << (st.opacity < 1.0 ? " stroke-opacity=\"" + fmt(st.opacity, 2) + "\"" : "") << "/>";
      } else if (e[i].kind == 1) {
        os << "<circle cx=\"" << sx + 11 << "\" cy=\"" << cy << "\" r=\"6\" fill=\"none\" stroke=\""
           << st.color << "\" stroke-width=\"" << st.width << "\""
           << (st.dash.empty() ? "" : " stroke-dasharray=\"" + st.dash + "\"") << "/>";
      } else {
        drawMarker(os, sx + 11, cy, st.color, e[i].shape);
      }
      os << "<text x=\"" << sx + 30 << "\" y=\"" << cy + 4 << "\" font-size=\"12\" fill=\""
         << kInk << "\">" << escape(e[i].label) << "</text>\n";
    }
  }

  std::vector<Line> lines_;
  std::vector<Circle> circles_;
  std::vector<Marker> markers_;
  std::vector<HLine> hlines_;
  std::vector<Text> texts_;
};

// One figure = one SVG file; axes are stacked top to bottom.
class Figure
{
public:
  Figure(double width, double height) : w_(width), h_(height) {}

  Axes & addAxes()
  {
    axes_.emplace_back();
    return axes_.back();
  }

  bool save(const std::string & path) const
  {
    std::ostringstream os;
    os << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << w_ << "\" height=\"" << h_
       << "\" viewBox=\"0 0 " << w_ << " " << h_ << "\">\n"
       << "<rect width=\"100%\" height=\"100%\" fill=\"" << kSurface << "\"/>\n";
    const double ah = h_ / std::max<size_t>(1, axes_.size());
    for (size_t i = 0; i < axes_.size(); ++i) {
      axes_[i].render(os, 0, ah * i, w_, ah, static_cast<int>(i));
    }
    os << "</svg>\n";
    std::ofstream f(path);
    f << os.str();
    return static_cast<bool>(f);
  }

private:
  double w_, h_;
  std::deque<Axes> axes_;  // deque: references returned by addAxes() stay valid
};

}  // namespace svgplot
