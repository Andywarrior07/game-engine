// //
// // Created by Andres Guerrero on 30-07-25.
// //
//
#include "Game.h"
//
// #include <iostream>
// #include <SDL2/SDL_image.h>
//
// SDL_Texture* player;
// SDL_Rect srcR, destR;
//
// Game::Game() {
// }
//
// Game::~Game() {
// }
//
// void Game::initialize(const char* title, const int x, const int y, const int width, const int height,
//                       const bool fullscreen) {
//     if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
//         std::cerr << "Error initializing SDL:" << SDL_GetError() << std::endl;
//         _isRunning = false;
//         return;
//     }
//
//     std::cout << "SDL initialized" << std::endl;
//
//     int flags = 0;
//
//     if (fullscreen) {
//         flags = SDL_WINDOW_FULLSCREEN;
//     }
//
//     window = SDL_CreateWindow(title, x, y, width, height, flags);
//
//     if (window == nullptr) {
//         std::cerr << "Error creating window: " << SDL_GetError() << std::endl;
//         _isRunning = false;
//         return;
//     }
//
//     renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
//
//     if (renderer == nullptr) {
//         std::cerr << "Error creating renderer: " << SDL_GetError() << std::endl;
//         _isRunning = false;
//         return;
//     }
//
//     if (!inputManager.initialize()) {
//         std::cerr << "Error initializing input manager: " << SDL_GetError() << std::endl;
//         _isRunning = false;
//         return;
//     }
//
//     setupInputBindings();
//
//     SDL_SetRenderDrawColor(renderer, 10, 255, 10, 255);
//
//     SDL_Surface* tempSurface = IMG_Load("../assets/player.png");
//     player = SDL_CreateTextureFromSurface(renderer, tempSurface);
//     SDL_FreeSurface(tempSurface);
//     SDL_SetTextureBlendMode(player, SDL_BLENDMODE_BLEND);
//
//
//     _isRunning = true;
// }
//
// void Game::processInput() {
//     inputManager.processEvents();
//     inputManager.update();
// }
//
// void Game::setupInputBindings() {
//     inputManager.bindKeyToAction(SDLK_ESCAPE, InputAction::EXIT);
//
//     inputManager.bindActionCallback(InputAction::EXIT, [this]() {
//         _isRunning = false;
//     });
// }
//
// void Game::update() {
//     destR.h = 64;
//     destR.w = 64;
//     destR.x = 100;
//     destR.y = 100;
// }
//
// void Game::render() {
//     SDL_RenderClear(renderer);
//
//     SDL_RenderCopy(renderer, player, nullptr, &destR);
//     SDL_RenderPresent(renderer);
// }
//
// void Game::clean() {
//     inputManager.shutdown();
//
//     std::cout << "Subsystems cleaned up" << std::endl;
//
//     SDL_DestroyWindow(window);
//     SDL_DestroyRenderer(renderer);
//     SDL_Quit();
//
//     std::cout << "SDL cleaned up" << std::endl;
// }
