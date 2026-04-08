#include <iostream>

#include "ball_sim.h"

int main() {
  if (!init_from_default(true)) {
    std::cerr << "Failed to initialize ball_sim. Run from repo root so "
                 "./collision_meshes/ is available.\n";
    return 1;
  }

  // Alternative init options:
  // - init_from_path("/path/to/collision_meshes", true/false)
  // - init_from_mem({ MeshFile{GameMode::Soccar, data}, ... }, true/false)
  // You can also check is_initialized() if you need a guard.

  auto arena = arena_new(GameMode::Soccar);

  auto state = ball_state_default();
  state.pos = {0.0f, 0.0f, 100.0f};
  state.vel = {200.0f, 100.0f, 500.0f};
  state.ang_vel = {1.0f, 2.0f, 3.0f};

  state.hs_info.y_target_dir = 0;
  state.hs_info.cur_target_speed = 1000.0f;
  state.hs_info.time_since_hit = 0.25f;

  state.ds_info.charge_level = 2;
  state.ds_info.accumulated_hit_force = 50.0f;
  state.ds_info.y_target_dir = 1;
  state.ds_info.has_damaged = false;
  state.ds_info.last_damage_tick = 0;

  state.last_extra_hit_tick_present = true;
  state.last_extra_hit_tick = 42;
  state.tick_count_since_kickoff = 5;

  arena->set_ball_state(state);
  arena->step_tick();
  arena->step_tick();

  auto out = arena->get_ball_state();
  std::cout << "pos: (" << out.pos.x << ", " << out.pos.y << ", " << out.pos.z
            << ")\n";
  std::cout << "vel: (" << out.vel.x << ", " << out.vel.y << ", " << out.vel.z
            << ")\n";
  std::cout << "ang_vel: (" << out.ang_vel.x << ", " << out.ang_vel.y << ", "
            << out.ang_vel.z << ")\n";

  std::cout << "hs_info.y_target_dir: "
            << static_cast<int>(out.hs_info.y_target_dir) << "\n";
  std::cout << "hs_info.cur_target_speed: " << out.hs_info.cur_target_speed
            << "\n";
  std::cout << "hs_info.time_since_hit: " << out.hs_info.time_since_hit << "\n";

  std::cout << "ds_info.charge_level: " << out.ds_info.charge_level << "\n";
  std::cout << "ds_info.accumulated_hit_force: "
            << out.ds_info.accumulated_hit_force << "\n";
  std::cout << "ds_info.y_target_dir: "
            << static_cast<int>(out.ds_info.y_target_dir) << "\n";
  std::cout << "ds_info.has_damaged: "
            << (out.ds_info.has_damaged ? "true" : "false") << "\n";
  std::cout << "ds_info.last_damage_tick: " << out.ds_info.last_damage_tick
            << "\n";

  std::cout << "last_extra_hit_tick_present: "
            << (out.last_extra_hit_tick_present ? "true" : "false") << "\n";
  std::cout << "last_extra_hit_tick: " << out.last_extra_hit_tick << "\n";
  std::cout << "tick_count_since_kickoff: " << out.tick_count_since_kickoff
            << "\n";

  return 0;
}
