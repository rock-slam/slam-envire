Vizkit::UiLoader.register_3d_plugin('EnvireVisualization', 'envire', 'EnvireVisualization')
Vizkit::UiLoader.register_3d_plugin_for('EnvireVisualization', "/envire/BinaryEvent", :updateBinaryEvent )
Vizkit::UiLoader.register_3d_plugin_for('EnvireVisualization', "/std/vector</envire/BinaryEvent>", :updateBinaryEvents )
Vizkit::UiLoader.register_3d_plugin_for('EnvireVisualization', "/RTT/extras/ReadOnlyPointer</std/vector</envire/BinaryEvent>>", :updateBinaryEvents )
