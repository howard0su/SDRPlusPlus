#include "ImOsmRichMarkStorage.h"
#include "ImOsmRichMarkItem.h"

namespace ImOsm {
namespace Rich {
MarkStorage::MarkStorage() = default;

MarkStorage::~MarkStorage() = default;

std::shared_ptr<MarkItem> MarkStorage::findMark(const std::string &name) const {
  const auto it{std::find_if(_markItems.begin(),
                             _markItems.end(),
                             [name](const ItemNode &node) {
                               return node.ptr->text() == name;
                             })};
  if (it != _markItems.end()) {
    return it->ptr;
  }
  return nullptr;
}

GeoCoords MarkStorage::findMark(const std::string &name, bool &ok) const {
  const auto ptr{findMark(name)};
  if (ptr) {
    ok = true;
    return ptr->geoCoords();
  }
  ok = false;
  return {};
}

void MarkStorage::addMark(const GeoCoords &coords, const std::string &name) {
  _markItems.push_back({std::make_shared<MarkItem>(coords, name)});
}

void MarkStorage::rmMarks() {
  _markItems.erase(std::remove_if(_markItems.begin(), _markItems.end(),
                                  [](auto &item) { return item.rmFlag; }),
                   _markItems.end());
}

bool MarkStorage::handleLoadState() {
  const bool loadState{_loadState};
  _loadState = false;
  return loadState;
}

bool MarkStorage::handlePickState() {
  const bool pickState{_pickState};
  _pickState = false;
  return pickState;
}

} // namespace Rich
} // namespace ImOsm
