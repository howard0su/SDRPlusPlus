#pragma once
#include <memory>
#include <string>

namespace ImOsm {
class MapPlot;
class ITileLoader;

class TileSourceWidget {
public:
  TileSourceWidget(std::shared_ptr<MapPlot> mapPlot);
  ~TileSourceWidget();

  void paint();

private:
  void updateTileLoader();

private:
  std::shared_ptr<MapPlot> _mapPlot;
  std::shared_ptr<ITileLoader> _tileLoader;

  struct Ui;
  std::unique_ptr<Ui> _ui;
};
} // namespace ImOsm
