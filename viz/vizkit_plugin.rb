Vizkit::UiLoader.register_3d_plugin('envire::EnvireVisualization', 'envire', 'EnvireVisualization')
Vizkit::UiLoader.register_3d_plugin_for('envire::EnvireVisualization', "/envire/BinaryEvent", :updateBinaryEvent )
Vizkit::UiLoader.register_3d_plugin_for('envire::EnvireVisualization', "/std/vector</envire/BinaryEvent>", :updateBinaryEvents )
