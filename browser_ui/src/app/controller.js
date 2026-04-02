import { createAppState } from "./state.js";
import { bootstrapSceneRenderer } from "../scene/sceneRenderer.js";
import { createMotionSystem } from "../scene/motionSystem.js";
import { createApiService } from "../services/api.js";
import { createRobotArmController } from "../robot/robotArmController.js";
import { createGantryController } from "../robot/gantryController.js";

export function createAppController() {
  const state = createAppState();
  const motionSystem = createMotionSystem();
  const api = createApiService();
  const robotArmController = createRobotArmController();
  const gantryController = createGantryController();

  return {
    start() {
      if (state.booted) {
        return;
      }
      state.booted = true;

      // Keep references on state so modules can be wired incrementally.
      state.modules = {
        motionSystem,
        api,
        robotArmController,
        gantryController,
      };

      bootstrapSceneRenderer();
    },
  };
}
