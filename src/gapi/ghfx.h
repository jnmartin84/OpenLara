#ifndef H_GAPI_HFX
#define H_GAPI_HFX

#include "core.h"

#define PROFILE_MARKER(title)
#define PROFILE_LABEL(id, name, label)
#define PROFILE_TIMING(time)

#define get_red_5551(color)      (((color    >>    11)    &    0x0000001F)<<3)
#define get_green_5551(color)    (((color    >>     6)    &    0x0000001F)<<3)
#define get_blue_5551(color)     (((color    >>     1)    &    0x0000001F)<<3)

#define COLOR_16
#define COLOR_FMT_555
#define CONV_COLOR(r,g,b) ((uint16_t)((((r >> 3)&0x1f) << 11) | (((g >> 3)&0x1f) << 6) | (((b >> 3)&0x1f)<<1) | 1))

extern "C" {
#include <hfx.h>
#include <hfx_rb.h>
#include <hfx_cmds.h>
#include <hfx_int.h>
#include <hfx_types.h>
void hfx_load_matrix(hfx_state *state, float *m);
}

namespace GAPI {

    using namespace Core;

    typedef ::Vertex Vertex;

    typedef uint16 ColorSW;
    typedef uint16 DepthSW;
    uint16_t texd[32*32];

    uint8_t colbuf[4*3000];
    float vertbuf[3*3000];
    float texbuf[2*3000];

    hfx_state *state;
    uint8   *swLightmap;
    uint8   swLightmapNone[32 * 256];
    uint8   swLightmapShade[32 * 256];
    ColorSW *swPalette;
    ColorSW swPaletteColor[256];
    ColorSW swPaletteWater[256];
    ColorSW swPaletteGray[256];
    uint8   swGradient[256];
    Tile8   *curTile;

    uint8 ambient;
    int32 lightsCount;

    struct LightSW {
        uint32 intensity;
        vec3   pos;
        float  radius;
    } lights[MAX_LIGHTS], lightsRel[MAX_LIGHTS];

// Shader
    struct Shader {
        void init(Pass pass, int type, int *def, int defCount) {}
        void deinit() {}
        void bind() {}
        void setParam(UniformType uType, const vec4  &value, int count = 1) {}
        void setParam(UniformType uType, const mat4  &value, int count = 1) {}
    };
	
// Texture
    struct Texture {
        uint8      *memory;
        int        width, height, origWidth, origHeight;
        TexFormat  fmt;
        uint32     opt;
        Texture(int width, int height, int depth, uint32 opt) : memory(0), width(width), height(height), origWidth(width), origHeight(height), fmt(FMT_RGBA), opt(opt) {}

        void init(void *data) {
            ASSERT((opt & OPT_PROXY) == 0);

            opt &= ~(OPT_CUBEMAP | OPT_MIPMAPS);

            //memory = new uint8[width * height * 4];
            if (data) {
                update(data);
            }
        }

        void deinit() {
            if (memory) {
                delete[] memory;
            }
        }

        void generateMipMap() {}

        void update(void *data) {
            //memcpy(memory, data, width * height * 4);
        }

        void bind(int sampler) {
            Core::active.textures[sampler] = this;

            if (!this || (opt & OPT_PROXY)) return;
            //ASSERT(memory);

            curTile = NULL;
        }

        void bindTileIndices(Tile8 *tile) {
            curTile = (Tile8*)tile;
#if 0
            for(uint32_t y=0;y<32;y++) {
                for(uint32_t x=0;x<32;x++) {
                    texd[(y*32) + x] = swPaletteColor[curTile->index[(((y+64)*2)*256)+((x+64)*2)]];
                }
            }

            hfx_tex_image_2d(state, 0, 0, HFX_RGBA, 32, 32, 0, HFX_RGBA, HFX_UNSIGNED_SHORT_5_5_5_1, &texd[0]);
#endif
        }

        void unbind(int sampler) {}

        void setFilterQuality(int value) {
            if (value > Settings::LOW)
                opt &= ~OPT_NEAREST;
            else
                opt |= OPT_NEAREST;
        }
    };

// Mesh
    struct Mesh {
        Index        *iBuffer;
        GAPI::Vertex *vBuffer;

        int          iCount;
        int          vCount;
        bool         dynamic;

        Mesh(bool dynamic) : iBuffer(NULL), vBuffer(NULL), dynamic(dynamic) {}

        void init(Index *indices, int iCount, ::Vertex *vertices, int vCount, int aCount) {
            this->iCount  = iCount;
            this->vCount  = vCount;

            iBuffer = new Index[iCount];
            vBuffer = new Vertex[vCount];

            update(indices, iCount, vertices, vCount);
        }

        void deinit() {
            delete[] iBuffer;
            delete[] vBuffer;
        }

        void update(Index *indices, int iCount, ::Vertex *vertices, int vCount) {
            if (indices) {
                __n64_memcpy_ASM(iBuffer, indices, iCount * sizeof(indices[0]));
            }

            if (vertices) {
                __n64_memcpy_ASM(vBuffer, vertices, vCount * sizeof(vertices[0]));
            }
        }

        void setupFVF() const {
	    //hfx_vertex_pointer(state, 3, HFX_FLOAT, 0, vertarr);// + (vstart*sizeof(float)*3));
	    //hfx_color_pointer(state, 4, HFX_UNSIGNED_BYTE, 0, colarr);// + (vstart*sizeof(uint8_t)*4));
        }

        void bind(const MeshRange &range) const {
            if (range.aIndex == -1) {
                setupFVF();
            }
        }

        void initNextRange(MeshRange &range, int &aIndex) const {
            range.aIndex = -1;
        }
    };

    int cullMode, blendMode;

    short4  swClipRect;

    struct VertexSW {
        int32 x, y, z, w;
        int32 u, v, l;

        inline VertexSW operator + (const VertexSW &p) const {
            VertexSW ret;
            ret.x = x + p.x;
            ret.y = y;
            ret.z = z + p.z;
            ret.w = w + p.w;
            ret.u = u + p.u;
            ret.v = v + p.v;
            ret.l = l + p.l;
            return ret;
        }

        inline VertexSW operator - (const VertexSW &p) const {
            VertexSW ret;
            ret.x = x - p.x;
            ret.y = y;
            ret.z = z - p.z;
            ret.w = w - p.w;
            ret.u = u - p.u;
            ret.v = v - p.v;
            ret.l = l - p.l;
            return ret;
        }

        inline VertexSW operator * (const int32 s) const {
            VertexSW ret;
            ret.x = x * s;
            ret.y = y;
            ret.z = z * s;
            ret.w = w * s;
            ret.u = u * s;
            ret.v = v * s;
            ret.l = l * s;
            return ret;
        }

        inline VertexSW operator / (const int32 s) const {
            VertexSW ret;
            ret.x = x / s;
            ret.y = y;
            ret.z = z / s;
            ret.w = w / s;
            ret.u = u / s;
            ret.v = v / s;
            ret.l = l / s;
            return ret;
        }
    };

    void init() {
        LOG("Renderer : %s\n", "libhfx");
        LOG("Version  : %s\n", "0.0");

        state = hfx_init();

        //hfx_enable(state, HFX_TEXTURE_2D);
    }

    void deinit() {
    }

    void resize() {
    }

    inline mat4::ProjRange getProjRange() {
        return mat4::PROJ_ZERO_POS;
    }

    mat4 ortho(float l, float r, float b, float t, float znear, float zfar) {
        mat4 m;
        m.ortho(getProjRange(), l, r, b, t, znear, zfar);
        return m;
    }

    mat4 perspective(float fov, float aspect, float znear, float zfar, float eye) {
        mat4 m;
        m.perspective(getProjRange(), fov, aspect, znear, zfar, eye);
        return m;
    }

    bool beginFrame() {
        return true;
    }

    void endFrame() {}

    void resetState() {}

    void bindTarget(Texture *texture, int face) {}

    void discardTarget(bool color, bool depth) {}

    void copyTarget(Texture *dst, int xOffset, int yOffset, int x, int y, int width, int height) {}

    void setVSync(bool enable) {}

    void waitVBlank() {}

    void clear(bool color, bool depth) {
        uint32_t bits = 0;
	if(depth) bits |= HFX_DEPTH_BUFFER_BIT;
	if(color) bits |= HFX_COLOR_BUFFER_BIT;
	hfx_clear(state, bits);
    }

    void setClearColor(const vec4 &color) {
        hfx_clear_color_f(state, color.x, color.y, color.z, 1.0f);
    }

    void setViewport(const short4 &v) {
    }

    void setScissor(const short4 &s) {
        swClipRect.x = s.x;
        swClipRect.y = Core::active.viewport.w - (s.y + s.w);
        swClipRect.z = s.x + s.z;
        swClipRect.w = Core::active.viewport.w - s.y;
    }

    void setDepthTest(bool enable) {
#if 1
        if(enable)
            hfx_enable(state, HFX_DEPTH_TEST);
         else
            hfx_disable(state, HFX_DEPTH_TEST);
#endif
    }

    void setDepthWrite(bool enable) {}

    void setColorWrite(bool r, bool g, bool b, bool a) {}

    void setAlphaTest(bool enable) {}

    void setCullMode(int rsMask) {
        cullMode = rsMask;
        switch (rsMask) {
            case RS_CULL_BACK  : hfx_cull_face(state, HFX_BACK);     break;
            case RS_CULL_FRONT : hfx_cull_face(state, HFX_FRONT);    break;
            default            : hfx_disable(state, HFX_CULL_FACE); return;
        }
        hfx_enable(state, HFX_CULL_FACE);
    }

    void setBlendMode(int rsMask) {}

    void setViewProj(const mat4 &mView, const mat4 &mProj) {}

    void updateLights(vec4 *lightPos, vec4 *lightColor, int count) {
        ambient = clamp(int32(active.material.y * 255), 0, 255);

        lightsCount = 0;
        for (int i = 0; i < count; i++) {
            if (lightColor[i].w >= 1.0f) {
                continue;
            }
            LightSW &light = lights[lightsCount++];
            vec4 &c = lightColor[i];
            light.intensity = uint32(((c.x + c.y + c.z) / 3.0f) * 255.0f);
            light.pos    = lightPos[i].xyz();
            light.radius = lightColor[i].w;
        }
    }

    void setFog(const vec4 &params) {}

    bool checkBackface(const VertexSW *a, const VertexSW *b, const VertexSW *c) {
        return ((b->x - a->x) >> 16) * (c->y - a->y) -
               ((c->x - a->x) >> 16) * (b->y - a->y) <= 0;
    }

    void applyLighting(VertexSW &result, const Vertex &vertex, float depth) {
#if 0
        vec3 coord  = vec3(float(vertex.coord.x), float(vertex.coord.y), float(vertex.coord.z));
        vec3 normal = vec3(float(vertex.normal.x), float(vertex.normal.y), float(vertex.normal.z)).normal();
        float lighting = 0.0f;
        for (int i = 0; i < lightsCount; i++) {
            LightSW &light = lightsRel[i];
            vec3 dir = (light.pos - coord) * light.radius;
            float att = dir.length2();
            float lum = normal.dot(dir / sqrtf(att));
            lighting += (max(0.0f, lum) * max(0.0f, 1.0f - att)) * light.intensity;
        }

        lighting += result.l;

        depth -= SW_FOG_START;
        if (depth > 0.0f) {
            lighting *= clamp(1.0f - depth / (SW_MAX_DIST - SW_FOG_START), 0.0f, 1.0f);
        }

        result.l = (255 - min(255, int32(lighting))) << 16;
#endif
    }

    bool transform(const Index *indices, const Vertex *vertices, int iStart, int iCount, int vStart) {
        return false;
    }

    void transformLights() {
        memcpy(lightsRel, lights, sizeof(LightSW) * lightsCount);

        mat4 mModelInv = mModel.inverseOrtho();
        for (int i = 0; i < lightsCount; i++) {
            lightsRel[i].pos = mModelInv * lights[i].pos;
        }
    }

    void DIP(Mesh *mesh, const MeshRange &range) {
        float v1[4], v2[4], v3[4], c1[4], c2[4], c3[4], t1[2], t2[2], t3[2];

	if (curTile == NULL) {
            return;
        }

        mat4 m = mProj * mView * mModel;

        hfx_matrix_mode(state, HFX_MODELVIEW);
        hfx_load_matrix_f(state, (float*)&m);

	int max = (range.iCount < 384 ? range.iCount : 384);
//	3000 ? range.iCount : 3000);

        hfx_set_mode(state);
        //for(int i = max-3; i > -1/*< max*/; i-=3)
        for(int i=0;i<max;i+=3)
        {
	    Vertex &vertex1 = mesh->vBuffer[mesh->iBuffer[(i+0)+range.iStart] + range.vStart];
	    Vertex &vertex2 = mesh->vBuffer[mesh->iBuffer[(i+1)+range.iStart] + range.vStart];
	    Vertex &vertex3 = mesh->vBuffer[mesh->iBuffer[(i+2)+range.iStart] + range.vStart];

            v1[0] = (float)vertex1.coord.x;
            v1[1] = (float)vertex1.coord.y;
            v1[2] = (float)vertex1.coord.z;
            v1[3] = 1.0f;

            v2[0] = (float)vertex2.coord.x;
            v2[1] = (float)vertex2.coord.y;
            v2[2] = (float)vertex2.coord.z;
            v2[3] = 1.0f;

        v3[0] = (float)vertex3.coord.x;
        v3[1] = (float)vertex3.coord.y;
        v3[2] = (float)vertex3.coord.z;
        v3[3] = 1.0f;

		uint16_t color1 = swPalette[curTile->index[(vertex1.texCoord.y*256)+(vertex1.texCoord.x)]];
		uint16_t color2 = swPalette[curTile->index[(vertex2.texCoord.y*256)+(vertex2.texCoord.x)]];
		uint16_t color3 = swPalette[curTile->index[(vertex3.texCoord.y*256)+(vertex3.texCoord.x)]];
		int cr1 = get_red_5551(color1);
		int cg1 = get_green_5551(color1);
		int cb1 = get_blue_5551(color1);
		int cr2 = get_red_5551(color2);
		int cg2 = get_green_5551(color2);
		int cb2 = get_blue_5551(color2);
		int cr3 = get_red_5551(color3);
		int cg3 = get_green_5551(color3);
		int cb3 = get_blue_5551(color3);
	
		c1[0] = (cr1 * vertex1.light.x)>>8; /// 256.0f;
		c1[1] = (cg1 * vertex1.light.x)>>8; // / 256.0f;
		c1[2] = (cb1 * vertex1.light.x)>>8; // / 256.0f;
		c1[3] = (color1&1)*255;

		c2[0] = (cr2 * vertex2.light.x)>>8; // / 256.0f;
		c2[1] = (cg2 * vertex2.light.x)>>8; // / 256.0f;
		c2[2] = (cb2 * vertex2.light.x)>>8; // / 256.0f;
		c2[3] = (color2&1)*255;
		
		c3[0] = (cr3 * vertex3.light.x)>>8; // / 256.0f;
		c3[1] = (cg3 * vertex3.light.x)>>8; // / 256.0f;
		c3[2] = (cb3 * vertex3.light.x)>>8; // / 256.0f;
		c3[3] = (color3&1)*255;

        t1[0] = t1[1] = t2[0] = t2[1] = t3[0] = t3[1] = 0.0f;

        hfx_draw_tri_f(state, v1, v2, v3, c1, c2, c3, t1, t2, t3);
    }
}

    void initPalette(Color24 *palette, uint8 *lightmap) {
        for (uint32 i = 0; i < 256; i++) {
            const Color24 &p = palette[i];
			if(i == 0) {
            swPaletteColor[i] = 0;//CONV_COLOR(p.r, p.g, p.b);//255);// & 0xffff;
            swPaletteWater[i] = 0;//CONV_COLOR((uint32(p.r) * 150) >> 8, (uint32(p.g) * 230) >> 8, (uint32(p.b) * 230) >> 8);//, 255);//&0xffff;
            swPaletteGray[i]  = 0;//CONV_COLOR((i * 57) >> 8, (i * 29) >> 8, (i * 112) >> 8);//, 255);//&0xffff;
            swGradient[i]     = 0; // all above were CONV_COLOR
			}
            swPaletteColor[i] = CONV_COLOR(p.r, p.g, p.b);//255);// & 0xffff;
            swPaletteWater[i] = CONV_COLOR((uint32(p.r) * 150) >> 8, (uint32(p.g) * 230) >> 8, (uint32(p.b) * 230) >> 8);//, 255);//&0xffff;
            swPaletteGray[i]  = CONV_COLOR((i * 57) >> 8, (i * 29) >> 8, (i * 112) >> 8);//, 255);//&0xffff;
            swGradient[i]     = i; // all above were CONV_COLOR
        }

        for (uint32 i = 0; i < 256 * 32; i++) {
            swLightmapNone[i]  = i % 256;
            swLightmapShade[i] = lightmap[i];
        }

        swLightmap = swLightmapShade;
        swPalette  = swPaletteColor;
    }

    void setPalette(ColorSW *palette) {
        swPalette = palette;
    }

    void setShading(bool enabled) {
        swLightmap = enabled ? swLightmapShade : swLightmapNone;
    }

    vec4 copyPixel(int x, int y) {
        return vec4(0.0f); // TODO: read from framebuffer
    }
}

#endif
