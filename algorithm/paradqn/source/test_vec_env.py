"""Test script for vectorized environment wrapper.

This script tests the SubprocVecEnv wrapper with multiple parallel environments.
"""

import numpy as np
from environments.robocup2d_hybrid_helios_v0 import RobocupEnv_Hybrid_Helios
from environments.vec_env import SubprocVecEnv


def test_vec_env(num_envs=2, num_steps=10):
    """Test vectorized environment with multiple parallel environments.

    Args:
        num_envs: Number of parallel environments to create
        num_steps: Number of steps to run for testing
    """
    print(f"Testing SubprocVecEnv with {num_envs} parallel environments...")

    # Create vectorized environment
    env_fns = [lambda: RobocupEnv_Hybrid_Helios() for _ in range(num_envs)]
    vec_env = SubprocVecEnv(env_fns)

    print(f"Created {len(vec_env)} parallel environments")
    print(f"Number of agents per environment: {vec_env.num_agents}")
    print(f"Players: {vec_env.players}")

    # Reset all environments
    print("\nResetting all environments...")
    observations_list, infos_list = vec_env.reset()
    print(f"Received observations from {len(observations_list)} environments")

    # Run some steps
    print(f"\nRunning {num_steps} steps...")
    for step in range(num_steps):
        # Collect random actions for all environments
        actions_list = []
        for env_idx in range(num_envs):
            observations = observations_list[env_idx]
            actions = {}

            for player in vec_env.players:
                # Select random action
                action_mask = observations[player]["action_mask"]
                valid_actions = np.where(action_mask)[0]

                if len(valid_actions) > 0:
                    action_idx = np.random.choice(valid_actions)
                    # Random action parameters
                    action_param = np.random.randn(vec_env.action_space_single["parameters"].shape[0])
                    actions[player] = (action_idx, action_param)
                else:
                    # Default action if no valid actions
                    actions[player] = (0, np.zeros(vec_env.action_space_single["parameters"].shape[0]))

            actions_list.append(actions)

        # Step all environments
        observations_list, rewards_list, terminations_list, truncations_list, infos_list = vec_env.step(actions_list)

        # Check results
        total_reward = sum([sum(rewards.values()) for rewards in rewards_list])
        num_done = sum([all(term.values()) or all(trunc.values())
                       for term, trunc in zip(terminations_list, truncations_list)])

        print(f"Step {step + 1}: Total reward = {total_reward:.3f}, Done envs = {num_done}/{num_envs}")

        # Reset done environments
        for env_idx in range(num_envs):
            if all(terminations_list[env_idx].values()) or all(truncations_list[env_idx].values()):
                print(f"  Environment {env_idx} finished, resetting...")
                vec_env.remotes[env_idx].send(('reset', None))
                observations_list[env_idx], _ = vec_env.remotes[env_idx].recv()

    # Close environments
    print("\nClosing environments...")
    vec_env.close()
    print("Test completed successfully!")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test vectorized environment")
    parser.add_argument("--num_envs", type=int, default=2, help="Number of parallel environments")
    parser.add_argument("--num_steps", type=int, default=10, help="Number of steps to test")

    args = parser.parse_args()

    test_vec_env(num_envs=args.num_envs, num_steps=args.num_steps)
