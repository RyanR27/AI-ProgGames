#include "vec.hpp"
#include "draw-triangle-pro.hpp"
#include "raylib-cpp.hpp"
#include "graph.hpp"
#include "graph-utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>

//Task 1 Mary 
#include "raylib.h" 

//Task 4 Mary
Sound clickSound;
void LoadSounds(){
    InitAudioDevice();
    clickSound = LoadSound("../../deps/raylib-cpp/examples/audio/resources/coin.wav");
}

void PlayClickSound() {
    PlaySound(clickSound);
}


std::vector<node_t> astar_pathfind(const Graph& g, node_t start, node_t goal)
{
  std::unordered_map<node_t, node_t> came_from;
  std::unordered_map<node_t, double> cost_so_far;
  a_star_search(g, start, goal, came_from, cost_so_far);
  std::vector<node_t> path = reconstruct_path(start, goal, came_from);
  return path;
}

unsigned int path_cost(const std::vector<node_t>& path)
{
  double dcost = 0;
  if (path.size() >= 2) // then we have some lines to draw
  {
    const int num_edges = path.size() - 1;
    for (int i = 0; i < num_edges; i++)
    {
      dcost = dcost + edge_info[std::pair{path[i], path[i+1]}];
    }
  }

  return static_cast<unsigned int>(dcost);
}

int main()
{
  //Task 4 Mary
  LoadSounds();

  const int w{ 2880/2 }, h{ 1620/2 }, half_w{ w/2 }, half_h{ h/2 }, gap{ w/8 };
  raylib::Window window{ w, h, "Pathfinder" };

  SetTargetFPS(60);

  Graph g;
  add_node(g, 'A', { half_w - gap, half_h });
  add_node(g, 'B', { half_w, half_h });
  add_node(g, 'C', { half_w, half_h - gap });
  add_node(g, 'D', { half_w, half_h + gap });
  add_node(g, 'E', { half_w + gap, half_h + gap });
  add_node(g, 'F', { half_w + gap, half_h });
  add_node(g, 'G', { half_w + (2 * gap), half_h - gap });
  add_double_edge(g, 'A', 'B');
  add_double_edge(g, 'B', 'C');
  add_double_edge(g, 'B', 'D');
  add_double_edge(g, 'C', 'A');
  add_double_edge(g, 'D', 'E');
  add_double_edge(g, 'D', 'A');
  add_double_edge(g, 'E', 'B');
  add_double_edge(g, 'B', 'F');
  add_double_edge(g, 'C', 'F');
  add_double_edge(g, 'C', 'G');
  add_double_edge(g, 'F', 'G');


  float t{60}; // time from int to flaot William

  std::vector<node_t> player_path{};
  node_t start = 'A';
  node_t end   = 'G';
  int tokens{2000}, score{}, high_score{}; // try with more/less tokens?

  //std::vector<node_t> path;
  player_path.push_back(start); //Task 5 William



  while (!window.ShouldClose()) // Detect window close button or ESC key
  {
    BeginDrawing();

    //William
    t -= GetFrameTime();

    ClearBackground(LIGHTGRAY);



    //Task 5 William
    if (player_path.size() >= 2)
    {
        for (int i = 1; i < player_path.size(); i++)
        {
            const node_t n1 = player_path[i - 1];
            const node_t n2 = player_path[i];
            const coord_t& coord_n1 = node_info[n1];
            const coord_t& coord_n2 = node_info[n2];
            DrawLineEx(coord_n1, coord_n2, line_thickness * 2, YELLOW); // William - Drawing yellow line using player path coordinates
        }
    }

    draw_graph(g);

    //Task 1 Mary
    DrawText(TextFormat("Score: %04i", score), 100, 80, 20, WHITE); // Mary 
    DrawText(TextFormat("Tokens: %04i", tokens), 100, 120, 20, WHITE); // Mary
    DrawText(TextFormat("High Score: %04i", high_score), 100, 160, 20, WHITE); // Mary
    DrawText(TextFormat("Timer(s): %02i", static_cast<int>(t)), 100, 200, 20, WHITE); //Mary and cast as int was Willam

    //Task 2 William
    DrawCircle(node_info[start].x, node_info[start].y, 12, GREEN); //William - Drawing green circle using start node position
    DrawCircle(node_info[end].x, node_info[end].y, 12, RED); //William - Drawing red circle using end node position


    //Task 9 Mary
    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON))
    {
        if (player_path.size() > 1)
        {
            PlayClickSound();

            node_t lastNode = player_path.back();
            node_t secondLastNode = player_path[player_path.size() - 2];

            tokens += g.cost(secondLastNode, lastNode);
            player_path.pop_back();
        }
    }


    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
    {
      if (auto opt = get_nearby_node(GetMousePosition()))
      {
        // *opt is a node_t
         // Willam
        auto nv = g.neighbors(player_path.back());

          for (auto currNeighbour : nv)
          {
              if (currNeighbour == *opt)
              {
                  tokens -= g.cost(player_path.back(), *opt);
                  player_path.push_back(*opt);
                  //tokens -= path_cost(player_path); *maybe use it later -- final node
                  
                  //task 4 Mary
                  PlayClickSound();

                  break;
              }
          }
      }
    }

    EndDrawing();
  }

  //Task 4 Mary
  UnloadSound(clickSound);
  CloseAudioDevice();

  return 0;
}

