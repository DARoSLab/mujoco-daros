#include "graphbuilderadapter.h"

#include "daros_yaml-cpp/parser.h"  // IWYU pragma: keep

namespace daros_YAML {
class GraphBuilderInterface;

void* BuildGraphOfNextDocument(Parser& parser,
                               GraphBuilderInterface& graphBuilder) {
  GraphBuilderAdapter eventHandler(graphBuilder);
  if (parser.HandleNextDocument(eventHandler)) {
    return eventHandler.RootNode();
  } else {
    return NULL;
  }
}
}
