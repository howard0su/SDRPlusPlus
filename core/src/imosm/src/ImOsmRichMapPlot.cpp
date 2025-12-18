#include "ImOsmRichMapPlot.h"
#include "ImOsmIRichItem.h"

namespace ImOsm {
namespace Rich {

RichMapPlot::RichMapPlot() {}

RichMapPlot::~RichMapPlot() = default;

void RichMapPlot::paintOverMap() {
  ImOsm::MapPlot::paintOverMap();

  _items.erase(std::remove_if(_items.begin(), _items.end(),
                              [](auto item) { return item.expired(); }),
               _items.end());

  std::for_each(_items.begin(), _items.end(), [this](auto ptr) {
    auto item{ptr.lock()};
    if (item->enabled() &&
        item->inBounds(minLat(), maxLat(), minLon(), maxLon()))
      item->paint();
  });
}

void RichMapPlot::addItem(std::weak_ptr<IRichItem> item) {
  _items.push_back(item);
}
} // namespace Rich
} // namespace ImOsm
