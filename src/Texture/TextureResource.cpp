// #include "TextureResource.h"
// #include <SDL_image.h>      // Para cargar diferentes formatos de imagen
// #include <iostream>         // Para logging de errores
// #include <cassert>          // Para debug assertions
//
// namespace engine::resources {
//
//     /**
//      * @brief Constructor de TextureResource
//      */
//     TextureResource::TextureResource(ResourceID id,
//                                    std::string path,
//                                    LoadPriority priority,
//                                    SDL_Texture* texture,
//                                    int width,
//                                    int height)
//         : Resource<TextureResource>(id, std::move(path), priority)  // Llamar constructor base
//         , sdlTexture_(texture)                                      // Almacenar textura SDL
//         , width_(width)                                             // Almacenar dimensiones
//         , height_(height)
//     {
//         // Si se proporciona una textura, marcar como cargada
//         if (sdlTexture_) {
//             setState(ResourceState::LOADED);
//         }
//     }
//
//     /**
//      * @brief Destructor - RAII cleanup de recursos SDL
//      */
//     TextureResource::~TextureResource() {
//         // Liberar textura SDL si existe
//         if (sdlTexture_) {
//             SDL_DestroyTexture(sdlTexture_);
//             sdlTexture_ = nullptr;
//         }
//     }
//
//     /**
//      * @brief Move constructor para transferencia eficiente de ownership
//      */
//     TextureResource::TextureResource(TextureResource&& other) noexcept
//         : Resource<TextureResource>(std::move(other))               // Move base class
//         , sdlTexture_(other.sdlTexture_)                           // Transfer ownership
//         , width_(other.width_)
//         , height_(other.height_)
//     {
//         // Resetear el objeto source para evitar double-delete
//         other.sdlTexture_ = nullptr;
//         other.width_ = 0;
//         other.height_ = 0;
//     }
//
//     /**
//      * @brief Move assignment operator
//      */
//     TextureResource& TextureResource::operator=(TextureResource&& other) noexcept {
//         if (this != &other) {
//             // Limpiar recursos actuales
//             if (sdlTexture_) {
//                 SDL_DestroyTexture(sdlTexture_);
//             }
//
//             // Move base class
//             Resource<TextureResource>::operator=(std::move(other));
//
//             // Transfer ownership
//             sdlTexture_ = other.sdlTexture_;
//             width_ = other.width_;
//             height_ = other.height_;
//
//             // Reset source object
//             other.sdlTexture_ = nullptr;
//             other.width_ = 0;
//             other.height_ = 0;
//         }
//         return *this;
//     }
//
//     /**
//      * @brief Obtener información detallada de la textura SDL
//      */
//     int TextureResource::getTextureInfo(Uint32* format, int* access, int* width, int* height) const {
//         if (!sdlTexture_) {
//             return -1;  // Textura no válida
//         }
//
//         // Usar SDL para obtener información de la textura
//         return SDL_QueryTexture(sdlTexture_, format, access, width, height);
//     }
//
//     /**
//      * @brief Calcular el uso aproximado de memoria de esta textura
//      */
//     std::size_t TextureResource::getMemorySize() const noexcept {
//         return calculateMemorySize();
//     }
//
//     /**
//      * @brief Recargar la textura desde disco
//      */
//     bool TextureResource::reload() {
//         // Para recargar, necesitaríamos acceso al renderer y factory
//         // En una implementación completa, esto requeriría un callback al factory
//         // Por simplicidad, retornamos false (reload no implementado)
//         return false;
//     }
//
//     /**
//      * @brief Helper para calcular memoria usada por la textura
//      */
//     std::size_t TextureResource::calculateMemorySize() const noexcept {
//         if (!sdlTexture_ || width_ <= 0 || height_ <= 0) {
//             return 0;
//         }
//
//         // Obtener formato de la textura
//         Uint32 format;
//         int access, w, h;
//         if (SDL_QueryTexture(sdlTexture_, &format, &access, &w, &h) != 0) {
//             // Si no podemos obtener info, estimamos con RGBA (4 bytes por píxel)
//             return width_ * height_ * 4;
//         }
//
//         // Calcular bytes por píxel basado en el formato SDL
//         int bytesPerPixel = 4;  // Default RGBA
//         switch (format) {
//             case SDL_PIXELFORMAT_RGB888:
//             case SDL_PIXELFORMAT_BGR888:
//                 bytesPerPixel = 3;
//                 break;
//             case SDL_PIXELFORMAT_RGBA8888:
//             case SDL_PIXELFORMAT_ABGR8888:
//             case SDL_PIXELFORMAT_BGRA8888:
//             case SDL_PIXELFORMAT_ARGB8888:
//                 bytesPerPixel = 4;
//                 break;
//             case SDL_PIXELFORMAT_RGB565:
//             case SDL_PIXELFORMAT_BGR565:
//                 bytesPerPixel = 2;
//                 break;
//             default:
//                 bytesPerPixel = 4;  // Safe default
//                 break;
//         }
//
//         return width_ * height_ * bytesPerPixel;
//     }
//
//     // ============================================================================
//     // SDL TEXTURE FACTORY IMPLEMENTATION
//     // ============================================================================
//
//     /**
//      * @brief Constructor del factory que requiere un renderer SDL
//      */
//     SDLTextureFactory::SDLTextureFactory(SDL_Renderer* renderer)
//         : renderer_(renderer)                   // Almacenar renderer para crear texturas
//         , useLinearFiltering_(true)             // Usar filtrado linear por defecto
//     {
//         // Verificar que el renderer sea válido
//         assert(renderer_ != nullptr && "SDL Renderer cannot be null");
//
//         // Verificar que SDL_image esté disponible
//         if (!isSDLImageAvailable()) {
//             std::cerr << "Warning: SDL_image not properly initialized. Some image formats may not load." << std::endl;
//         }
//     }
//
//     /**
//      * @brief Crear TextureResource desde archivo de imagen
//      */
//     std::unique_ptr<TextureResource> SDLTextureFactory::createResource(ResourceID id,
//                                                                       const std::string& path,
//                                                                       LoadPriority priority) {
//         // 1. CONFIGURAR FILTRADO DE TEXTURA
//         setupTextureFiltering(useLinearFiltering_);
//
//         // 2. CARGAR IMAGEN DESDE DISCO USANDO SDL_IMAGE
//         SDL_Surface* surface = IMG_Load(path.c_str());
//         if (!surface) {
//             // Error al cargar imagen
//             std::cerr << "Failed to load image: " << path << " - " << IMG_GetError() << std::endl;
//             return nullptr;
//         }
//
//         // 3. OBTENER DIMENSIONES DE LA IMAGEN
//         int width = surface->w;
//         int height = surface->h;
//
//         // 4. CREAR TEXTURA SDL DESDE LA SURFACE
//         SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
//
//         // 5. LIBERAR SURFACE (ya no la necesitamos)
//         SDL_FreeSurface(surface);
//
//         if (!texture) {
//             // Error al crear textura
//             std::cerr << "Failed to create texture from surface: " << path << " - " << SDL_GetError() << std::endl;
//             return nullptr;
//         }
//
//         // 6. CREAR Y RETORNAR RESOURCE OBJECT
//         auto textureResource = std::make_unique<TextureResource>(id, path, priority, texture, width, height);
//
//         std::cout << "Successfully loaded texture: " << path << " (" << width << "x" << height << ")" << std::endl;
//
//         return textureResource;
//     }
//
//     /**
//      * @brief Obtener extensiones soportadas por SDL_image
//      */
//     std::vector<std::string> SDLTextureFactory::getSupportedExtensions() const {
//         // Retornar formatos comúnmente soportados por SDL_image
//         // En una implementación más robusta, podríamos verificar dinámicamente
//         // qué formatos están disponibles usando IMG_Init flags
//         return {
//             ".png",         // PNG (soportado por defecto)
//             ".jpg",         // JPEG
//             ".jpeg",        // JPEG (extensión alternativa)
//             ".bmp",         // Bitmap (soportado por SDL core)
//             ".tga",         // Targa
//             ".gif",         // GIF
//             ".webp",        // WebP (si está compilado el soporte)
//             ".tiff",        // TIFF
//             ".tif"          // TIFF (extensión alternativa)
//         };
//     }
//
//     /**
//      * @brief Verificar disponibilidad de SDL_image
//      */
//     bool SDLTextureFactory::isSDLImageAvailable() {
//         // Intentar inicializar SDL_image con formatos básicos
//         int imgFlags = IMG_INIT_PNG | IMG_INIT_JPG;
//         int initted = IMG_Init(imgFlags);
//
//         // Verificar si se inicializaron los formatos que queríamos
//         return (initted & imgFlags) == imgFlags;
//     }
//
//     /**
//      * @brief Configurar hints de filtrado de texturas
//      */
//     void SDLTextureFactory::setupTextureFiltering(bool linear) const {
//         // Configurar hint SDL para filtrado de texturas
//         if (linear) {
//             // Filtrado linear (suavizado) - mejor para texturas escaladas
//             SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
//         } else {
//             // Filtrado nearest (pixelated) - mejor para pixel art
//             SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
//         }
//
//         // Hint adicional para mejor calidad en algunos drivers
//         SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
//     }
//
// } // namespace engine::resources