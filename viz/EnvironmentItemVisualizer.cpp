#include "EnvironmentItemVisualizer.hpp"
#include "EnvireEventListener.hpp"

using namespace vizkit;

void EnvironmentItemVisualizer::updateVisualizedItems()
{
    if(event_listener)
        event_listener->updateItemsHandledBy(this);
}

void EnvironmentItemVisualizer::setEventListener(EnvireEventListener *listener)
{
    event_listener = listener;
}
