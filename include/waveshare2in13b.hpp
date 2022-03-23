#pragma once
#include <Arduino.h>
#include <gfx_bitmap.hpp>
#include <htcw_data.hpp>
#include <tft_driver.hpp>

namespace arduino {
    template<typename PixelType>
    struct waveshare2in13b_palette {
    private:
        constexpr static gfx::gfx_result index_to_mapped(int idx,PixelType* result) {
            uint8_t red = 31*(idx>0), green=63*(idx==2), blue=green>>1;
            gfx::convert(gfx::rgb_pixel<16>(red,green,blue),result);
            return gfx::gfx_result::success;
        }
    public:
        using type = waveshare2in13b_palette;
        using pixel_type = gfx::pixel<gfx::channel_traits<gfx::channel_name::index,2,0,2>>;
        using mapped_pixel_type = PixelType;
        constexpr static const bool writable = false;
        constexpr static const size_t size = 3;
        gfx::gfx_result map(pixel_type pixel,mapped_pixel_type* mapped_pixel) const {
            return index_to_mapped(pixel.channel<gfx::channel_name::index>(),mapped_pixel);
        }
        gfx::gfx_result nearest(mapped_pixel_type mapped_pixel,pixel_type* pixel) const {
            if(nullptr==pixel) {
                return gfx::gfx_result::invalid_argument;
            }
            mapped_pixel_type mpx;
            gfx::gfx_result r = index_to_mapped(0,&mpx);
            if(gfx::gfx_result::success!=r) {
                return r;
            }
            double least = mpx.difference(mapped_pixel);
            if(0.0==least) {
                pixel->native_value = 0;
                return gfx::gfx_result::success;
            }
            int ii=0;
            for(int i = 1;i<size;++i) {
                r=index_to_mapped(i,&mpx);
                if(gfx::gfx_result::success!=r) {
                    return r;
                }
                double cmp = mpx.difference(mapped_pixel);
                if(0.0==cmp) {
                    ii=i;
                    least = 0.0;
                    break;
                }
                if(cmp<least) {
                    least = cmp;
                    ii=i;
                }
            }
     
            pixel->channel<gfx::channel_name::index>(ii);
            return gfx::gfx_result::success;
        }
    };
    namespace waveshare2in13b_helpers {
        template<size_t DitherBitDepth> struct dither_types_impl {
            using pixel_type = gfx::rgb_pixel<DitherBitDepth>;
            using frame_buffer_type = gfx::large_bitmap<pixel_type>;
        };
        template<> struct dither_types_impl<0> {
            using pixel_type = gfx::pixel<gfx::channel_traits<gfx::channel_name::index,2,0,2>>;
            using frame_buffer_type = gfx::large_bitmap<pixel_type,waveshare2in13b_palette<gfx::rgb_pixel<16>>>;
        };
        struct build_pixel_info {
            data::simple_fixed_map<int,gfx::helpers::dither_color::mixing_plan_data_fast,100>* cache;
            uint8_t b;
            int d;
            int p;
            int i;
            int x;
            int y;
        };
        template<typename FrameBufferType,bool DitherBitDepth> struct build_pixel_impl {
            inline static uint8_t build_pixel(const build_pixel_info& info, typename waveshare2in13b_palette<gfx::rgb_pixel<16>>::pixel_type pixel) {
                return info.b;
            }
            inline static uint8_t build_pixel(const build_pixel_info& info, typename FrameBufferType::pixel_type pixel) {
                uint8_t b;
                if(!info.d) {
                    typename waveshare2in13b_palette<typename FrameBufferType::pixel_type>::pixel_type ppx;
                    waveshare2in13b_palette<typename FrameBufferType::pixel_type> palette;
                    palette.nearest(pixel,&ppx);
                    int px = ppx.template channel<gfx::channel_name::index>();
                    b=info.b|((1<<(7-info.i))*((((info.p==1)*(px!=1)))|((info.p==0)*(px>0))));
                    return b;
                }
                double map_value = gfx::helpers::dither_color::threshold_map_fast[(info.x & 7) + ((info.y & 7) << 3)];
                gfx::helpers::dither_color::mixing_plan_data_fast plan;
                if(info.cache!=nullptr) {
                    auto mp = info.cache->find(pixel.native_value);
                    if(mp!=nullptr) {
                        int px = mp->colors[map_value<mp->ratio?0:1];
                        b=info.b|((1<<(7-info.i))*((((info.p==1)*(px!=1)))|((info.p==0)*(px>0))));
                        return b;
                    }
                }
                waveshare2in13b_palette<typename FrameBufferType::pixel_type> palette;
                gfx::helpers::dither_color::mixing_plan_fast(&palette,pixel,&plan);
                if(info.cache!=nullptr) {
                    info.cache->insert({pixel.native_value,plan});
                }
                int px = plan.colors[map_value<plan.ratio?0:1];
                b=info.b|((1<<(7-info.i))*((((info.p==1)*(px!=1)))|((info.p==0)*(px>0))));
                return b;
            }
        };
        template<typename FrameBufferType> struct build_pixel_impl<FrameBufferType,0> {
            inline static uint8_t build_pixel(const build_pixel_info& info, typename FrameBufferType::pixel_type pixel) {
                int px =pixel.template channel<gfx::channel_name::index>();
                return info.b|((1<<(7-info.i))*((((info.p==1)*(px!=1)))|((info.p==0)*(px>0))));
            }
        };
        

    }

    template<int8_t PinDC, 
            int8_t PinRst, 
            int8_t PinWait, 
            typename Bus,
            size_t DitherBitDepth=0,
            unsigned int WriteSpeedPercent = 200>
    struct waveshare2in13b final {
        constexpr static const uint16_t width = 104;
        constexpr static const uint16_t height = 212;
        constexpr static const int8_t pin_dc = PinDC;
        constexpr static const int8_t pin_rst = PinRst;
        constexpr static const int8_t pin_wait = PinWait;
        constexpr static const float write_speed_multiplier = (WriteSpeedPercent/100.0);
        constexpr static const bool dithered = DitherBitDepth>1;
        using bus = Bus;
        using bus_driver = tft_driver<pin_dc,pin_rst,-1,bus>;
        using pixel_type = typename waveshare2in13b_helpers::dither_types_impl<DitherBitDepth>::pixel_type;
        using palette_type = waveshare2in13b_palette<gfx::rgb_pixel<16>>;
        using caps = gfx::gfx_caps<false,false,false,false,true,true,false>;
        using frame_buffer_type = typename waveshare2in13b_helpers::dither_types_impl<DitherBitDepth>::frame_buffer_type;
    private:
        void*(*m_allocator)(size_t);
        void*(*m_reallocator)(void*,size_t);
        void(*m_deallocator)(void*);
        frame_buffer_type m_frame_buffer;
        int m_suspend_count;
        gfx::rect16 m_suspend_bounds;
        bool m_sleep;
        bool m_dithering;
        bool m_initialized;
        palette_type m_palette;
        waveshare2in13b(const waveshare2in13b& rhs)=delete;
        waveshare2in13b& operator=(const waveshare2in13b& rhs)=delete;
        static void busy_low() { 
            while(!digitalRead(pin_wait)) delay(10);
        }
        static int hash_pixel(const int& pixel) {
            return pixel;
        }
        gfx::gfx_result update_display() {
            using cache_type = data::simple_fixed_map<int,gfx::helpers::dither_color::mixing_plan_data_fast,100>;
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            waveshare2in13b_helpers::build_pixel_info info;
            if(dithered) {
                info.d = m_dithering;
                waveshare2in13b_palette<pixel_type> palette;
                r=gfx::helpers::dither_color::prepare(&palette);
                if(r!=gfx::gfx_result::success) {
                    return r;
                }
                info.cache = (cache_type*)m_allocator(sizeof(cache_type));
                if(info.cache!=nullptr) {
                    *info.cache = cache_type(hash_pixel,m_allocator,m_reallocator,m_deallocator);
                }
                
            } else {
                info.cache = nullptr;
                info.d = false;
            }
            info.p=0;
            // do black plane
            bus_driver::send_command(0x10);
            delay(2);
            for(int y = 0;y<height;++y) {
                info.y = y;
                for(int ix = 0;ix<width/8;++ix) {
                    info.b = 0;
                    uint8_t b;
                    for(int bx = 0;bx<8;++bx) {
                        info.i = bx;
                        info.x = ix+bx;
                        pixel_type px;
                        m_frame_buffer.point({uint16_t(ix*8+bx),uint16_t(y)},&px);
                        b=waveshare2in13b_helpers::build_pixel_impl<frame_buffer_type,DitherBitDepth>::build_pixel(info,px);
                        info.b=b;
                    }
                    bus_driver::send_data8(b);
                }
            }
            delay(2);
            info.p=1;
            // do red  plane
            bus_driver::send_command(0x13);
            delay(2);
            for(int y = 0;y<height;++y) {
                info.y = y;
                for(int ix = 0;ix<width/8;++ix) {
                    info.b = 0;
                    uint8_t b;
                    for(int bx = 0;bx<8;++bx) {
                        info.i = bx;
                        info.x = ix+bx;
                        pixel_type px;
                        m_frame_buffer.point({uint16_t(ix*8+bx),uint16_t(y)},&px);
                        b=waveshare2in13b_helpers::build_pixel_impl<frame_buffer_type,DitherBitDepth>::build_pixel(info,px);
                        info.b=b;
                    }
                    bus_driver::send_data8(b);
                }
            }
            delay(2);
            if(dithered) {
                if(info.cache!=nullptr) {
                    info.cache->clear();
                    info.cache->~cache_type();
                    m_deallocator(info.cache);
                    // info.cache = nullptr;
                }
                gfx::helpers::dither_color::unprepare();
            }
            bus_driver::send_command(0x12);
            busy_low();
            
            
            return gfx::gfx_result::success;
        }
        
        static void expand_rect(gfx::rect16& dst,const gfx::rect16& src) {
            if(dst.x1==uint16_t(-1)) {
                dst=src;
            } else {
                if(src.x1<dst.x1) {
                    dst.x1 = src.x1;
                }
                if(src.x2>dst.x2) {
                    dst.x2 = src.x2;
                }
                if(src.y1<dst.y1) {
                    dst.y1 = src.y1;
                }
                if(src.y2>dst.y2) {
                    dst.y2 = src.y2;
                }
            }
        }
        public:
        waveshare2in13b(waveshare2in13b&& rhs) {
            m_allocator = rhs.m_allocator;
            m_reallocator = rhs.m_reallocator;
            m_deallocator = rhs.m_deallocator;
            m_frame_buffer = rhs.m_frame_buffer;
            m_suspend_count = rhs.m_suspend_count;
            m_suspend_bounds = rhs.m_suspend_bounds;
            m_sleep = rhs.m_sleep;
            m_dithering = rhs.m_dithering;
            m_initialized = rhs.m_initialized;
        }
        waveshare2in13b& operator=(waveshare2in13b&& rhs) {
            deinitialize();
            m_allocator = rhs.m_allocator;
            m_reallocator = rhs.m_reallocator;
            m_deallocator = rhs.m_deallocator;
            m_frame_buffer = rhs.m_frame_buffer;
            m_suspend_count = rhs.m_suspend_count;
            m_suspend_bounds = rhs.m_suspend_bounds;
            m_sleep = rhs.m_sleep;
            m_dithering = rhs.m_dithering;
            m_initialized = rhs.m_initialized;
            return *this;
        }
        inline gfx::size16 dimensions() const { return {width,height}; }
        inline gfx::rect16 bounds() const { return dimensions().bounds(); }
        gfx::gfx_result initialize() {
            if(!m_initialized || m_sleep) {
                if(!m_frame_buffer.initialized()) {
                    return gfx::gfx_result::out_of_memory;
                }
                if(!m_initialized) {
                    bus_driver::initialize();
                    pinMode(pin_wait, INPUT);
                    pinMode(pin_rst, OUTPUT);
                }
                reset();
                bus_driver::send_command(0x06);
                bus_driver::send_data8(0x17);
                bus_driver::send_data8(0x17);
                bus_driver::send_data8(0x17);
                bus_driver::send_command(0x04);
                busy_low();
                bus_driver::send_command(0x00);
                bus_driver::send_data8(0x8F);
                bus_driver::send_command(0x50);
                bus_driver::send_data8(0x37);
                bus_driver::send_command(0x61);
                bus_driver::send_data8(width & 0xFF);
                bus_driver::send_data8((width>>8)&0xFF);
                bus_driver::send_data8(height);
                m_sleep=false;
                m_initialized=true;
            }
            return gfx::gfx_result::success;
        }
        void deinitialize() {
            m_sleep=false;
            if(m_initialized) {
                bus_driver::deinitialize();
                m_initialized=false;
            }
        }
        inline bool initialized() const { return m_initialized; }
        gfx::gfx_result sleep() {
            if(!initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            if(!m_sleep) {
                bus_driver::send_command(0x2);
                busy_low();
                bus_driver::send_command(0x7);
                bus_driver::send_data8(0xa5);
                m_sleep=true;
            }
            return gfx::gfx_result::success;
        }
        void reset() {
            digitalWrite(pin_rst, LOW);
            delay(200);
            digitalWrite(pin_rst, HIGH);
            delay(200);
        }
        waveshare2in13b(void*(allocator)(size_t)=::malloc,void*(reallocator)(void*,size_t)=::realloc,void(deallocator)(void*)=::free) :
                m_allocator(allocator),
                m_reallocator(reallocator),
                m_deallocator(deallocator),
                m_frame_buffer(dimensions(),1,nullptr,allocator,deallocator),
                m_suspend_count(0),
                m_suspend_bounds(uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)),
                m_sleep(false),
                m_dithering(dithered),
                m_initialized(false) {
        }
        ~waveshare2in13b() {
            deinitialize();
        }
        inline bool dithering() const {
            return dithered && m_dithering;
        }
        inline void dithering(bool value) {
            m_dithering = value;
        }
        const palette_type* palette() const {
            return &m_palette;
        }
        gfx::gfx_result suspend() {
            ++m_suspend_count;
            return gfx::gfx_result::success;
        }
        gfx::gfx_result resume(bool force=false) {
            int os = m_suspend_count;
            if(force || 0==--m_suspend_count) {
                m_suspend_count = 0;
                gfx::gfx_result r = update_display(); 
                m_suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
                if(r!=gfx::gfx_result::success) {
                    m_suspend_count=os; // undo decrement
                    return r;
                }
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result point(gfx::point16 location, pixel_type color) {
            if(!bounds().intersects(location)) {
                return gfx::gfx_result::success;
            }
            gfx::gfx_result r = initialize();
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            expand_rect(m_suspend_bounds,{location.x,location.y,location.x,location.y});
            m_frame_buffer.point(location,color);
            if(!m_suspend_count) {
                r= update_display();
                if(r!=gfx::gfx_result::success) {
                    return r;
                }
                m_suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
            }
            return gfx::gfx_result::success;
        }
        inline gfx::gfx_result clear(const gfx::rect16& bounds) {
            pixel_type px;
            return fill(bounds,px);
        }
        gfx::gfx_result fill(const gfx::rect16& bounds,pixel_type color) {
            if(!this->bounds().intersects(bounds)) {
                return gfx::gfx_result::success;
            }
            gfx::rect16 rr = bounds.normalize().crop(this->bounds());
            gfx::gfx_result r = initialize();
            
            if(r!=gfx::gfx_result::success) {
                return r;
            }
            expand_rect(m_suspend_bounds,rr);
            m_frame_buffer.fill(bounds,color);
            if(!m_suspend_count) {
                r= update_display();
                if(r!=gfx::gfx_result::success) {
                    return r;
                }
                m_suspend_bounds = {uint16_t(-1),uint16_t(-1),uint16_t(-1),uint16_t(-1)};
            }
            return gfx::gfx_result::success;
        }
        gfx::gfx_result point(gfx::point16 location,pixel_type* out_color) const {
            if(!m_frame_buffer.initialized()) {
                return gfx::gfx_result::invalid_state;
            }
            if(!out_color) {
                return gfx::gfx_result::invalid_argument;
            }
            if(!this->bounds().intersects(location)) {
                return gfx::gfx_result::success;
            }
            return m_frame_buffer.point(location,out_color);
        }
    };
}