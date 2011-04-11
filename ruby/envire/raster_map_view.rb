require 'vizkit'
module RasterMapView
    attr_reader :map
    attr_reader :band

    vizkit_subclass_of 'ImageView'
    def self.setup(w)
        w.setAspectRatio(true)
        w.setZoomEnabled(true)
        w.openGL(false)
    end

    def load(map_path)
        @band = 0
        # Displays the map in image_view
        self.map = Gdal::Gdal.open(map_path)
    end

    def map=(map)
        @map = map
        update
    end

    def band=(index)
        @band = index
        update
    end

    def viewToMap(x, y)
        y = map.ysize - y
        map.apply_geo_transform(x, y)
    end

    def update
        data = map.read_band(1, 0, 0, map.xsize, map.ysize)
        min, max = data.min, data.max
        # WARN: we want to display maps with the Y axis going UP, not DOWN as on
        # normal images
        #
        # Therefore, pixels is reversed on a per-line basis before it gets
        # packed (just before addRawImage)
        pixels = []
        map.ysize.times do |y|
            line = []
            map.xsize.times do |x|
                v = Integer(255.0 * data.shift / max)
                if v == 0
                    line << 0 << 0 << 0
                else
                    line << (255 - v) << v << 125
                end
            end
            pixels << line
        end
        pixels = pixels.reverse.flatten.pack("C*")
        addRawImage("MODE_RGB", 3, map.xsize, map.ysize, pixels)
        super
    end
end

Vizkit::UiLoader.register_ruby_widget 'RasterMapView', RasterMapView.method(:new)
