/**
 * Base Controller Interface
 *
 * All robot controllers should extend this class.
 *
 * 统一使用异步模式：
 * - step() 方法每个控制周期调用一次（由 decimation 控制频率，默认 50Hz）
 * - 物理引擎在每个控制周期内执行多个子步（decimation 次）
 * - 控制器在 step() 中设置目标，物理引擎通过 actuator 执行
 */

export class BaseController {
  constructor() {
    this.initialized = false;
  }

  /**
   * Initialize the controller with model and data
   * 可以是异步的，用于加载控制器所需的外部资源
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} mujoco - MuJoCo WASM module
   */
  async initialize(model, data, mujoco) {
    throw new Error('initialize() must be implemented by subclass');
  }

  /**
   * Reset the controller to initial state
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   */
  reset(model, data) {
    throw new Error('reset() must be implemented by subclass');
  }

  /**
   * 异步控制步进 - 每个控制周期调用一次
   * 在这里读取键盘状态、计算控制量、设置 data.ctrl
   * @param {object} keyStates - Current keyboard states
   * @param {object} model - MuJoCo model
   * @param {object} data - MuJoCo data
   * @param {object} mujoco - MuJoCo WASM module
   */
  async step(keyStates, model, data, mujoco) {
    throw new Error('step() must be implemented by subclass');
  }

  /**
   * Get the list of keys this controller uses
   * @returns {string[]}
   */
  getControlKeys() {
    throw new Error('getControlKeys() must be implemented by subclass');
  }

  /**
   * Get description for GUI display
   * @returns {string}
   */
  getDescription() {
    throw new Error('getDescription() must be implemented by subclass');
  }
}
