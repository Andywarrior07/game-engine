// #pragma once
//
// #include "../Resource/ResourceManager.h"
// #include <SDL2/SDL.h>
// #include <string>
// #include <memory>
//
// namespace engine::resources {
//
//     /**
//      * @brief Texture resource implementation using SDL2
//      *
//      * Encapsula una textura SDL para uso con el ResourceManager.
//      * Maneja automáticamente la limpieza de recursos SDL en el destructor.
//      */
//     class TextureResource : public Resource<TextureResource> {
//     public:
//         /**
//          * @brief Constructor para TextureResource
//          * @param id Identificador único del recurso
//          * @param path Ruta del archivo de imagen
//          * @param priority Prioridad de carga
//          * @param texture Puntero a la textura SDL (puede ser nullptr inicialmente)
//          * @param width Ancho de la textura en píxeles
//          * @param height Alto de la textura en píxeles
//          */
//         TextureResource(ResourceID id,
//                        std::string path,
//                        LoadPriority priority,
//                        SDL_Texture* texture = nullptr,
//                        int width = 0,
//                        int height = 0);
//
//         /**
//          * @brief Destructor - libera automáticamente la textura SDL
//          */
//         ~TextureResource();
//
//         // No copiable pero movible para performance
//         TextureResource(const TextureResource&) = delete;
//         TextureResource& operator=(const TextureResource&) = delete;
//         TextureResource(TextureResource&& other) noexcept;
//         TextureResource& operator=(TextureResource&& other) noexcept;
//
//         /**
//          * @brief Obtener la textura SDL para rendering
//          * @return Puntero a SDL_Texture, o nullptr si no está cargada
//          */
//         SDL_Texture* getSDLTexture() const noexcept {
//             return sdlTexture_;
//         }
//
//         /**
//          * @brief Obtener el ancho de la textura
//          * @return Ancho en píxeles
//          */
//         int getWidth() const noexcept {
//             return width_;
//         }
//
//         /**
//          * @brief Obtener el alto de la textura
//          * @return Alto en píxeles
//          */
//         int getHeight() const noexcept {
//             return height_;
//         }
//
//         /**
//          * @brief Obtener información de formato de la textura
//          * @param format Puntero donde almacenar el formato SDL
//          * @param access Puntero donde almacenar el tipo de acceso
//          * @param width Puntero donde almacenar el ancho
//          * @param height Puntero donde almacenar el alto
//          * @return 0 en éxito, código de error SDL en fallo
//          */
//         int getTextureInfo(Uint32* format, int* access, int* width, int* height) const;
//
//         /**
//          * @brief Verificar si la textura es válida y está lista para usar
//          * @return true si la textura SDL es válida
//          */
//         bool isValid() const noexcept {
//             return sdlTexture_ != nullptr;
//         }
//
//         // Implementar interfaz IResource
//         std::size_t getMemorySize() const noexcept override;
//         bool reload() override;
//
//     private:
//         SDL_Texture* sdlTexture_;   // Puntero a la textura SDL (managed resource)
//         int width_;                 // Ancho de la textura en píxeles
//         int height_;                // Alto de la textura en píxeles
//
//         // Helper para calcular el tamaño de memoria aproximado
//         std::size_t calculateMemorySize() const noexcept;
//     };
//
//     /**
//      * @brief Factory para crear TextureResource usando SDL2
//      *
//      * Esta factory maneja la carga de imágenes desde disco usando SDL_image
//      * y crea texturas SDL compatibles con el renderer actual.
//      */
//     class SDLTextureFactory : public IResourceFactory<TextureResource> {
//     public:
//         /**
//          * @brief Constructor que requiere un renderer SDL válido
//          * @param renderer Renderer SDL que se usará para crear texturas
//          */
//         explicit SDLTextureFactory(SDL_Renderer* renderer);
//
//         /**
//          * @brief Destructor virtual
//          */
//         virtual ~SDLTextureFactory() = default;
//
//         /**
//          * @brief Crear una TextureResource desde un archivo de imagen
//          * @param id Identificador único para la textura
//          * @param path Ruta del archivo de imagen
//          * @param priority Prioridad de carga para memory management
//          * @return Unique_ptr a TextureResource creada, o nullptr si falla
//          */
//         std::unique_ptr<TextureResource> createResource(ResourceID id,
//                                                        const std::string& path,
//                                                        LoadPriority priority) override;
//
//         /**
//          * @brief Obtener extensiones de archivo soportadas
//          * @return Vector con extensiones soportadas por SDL_image
//          */
//         std::vector<std::string> getSupportedExtensions() const override;
//
//         /**
//          * @brief Configurar filtros de textura por defecto
//          * @param linear true para filtrado linear, false para nearest
//          */
//         void setDefaultFiltering(bool linear) { useLinearFiltering_ = linear; }
//
//         /**
//          * @brief Verificar si SDL_image está inicializado correctamente
//          * @return true si SDL_image puede cargar los formatos básicos
//          */
//         static bool isSDLImageAvailable();
//
//     private:
//         SDL_Renderer* renderer_;        // Renderer SDL para crear texturas
//         bool useLinearFiltering_;       // Usar filtrado linear por defecto
//
//         // Helper para configurar hints de filtrado
//         void setupTextureFiltering(bool linear) const;
//     };
//
// } // namespace engine::resources