"""Readable episode plans for the terminal before the simulator starts."""


def print_story(story, status="DRAFT"):
    print(f"SEMANTIC_SCENARIO_{status} id={story['scenario_id']}", flush=True)
    print(f"  Story: {story['summary']}", flush=True)
    for index, event in enumerate(story["events"], 1):
        print(f"  Event {index}: {event['description']}", flush=True)


def print_episode_plan(index, story, participants, plan):
    print(f"SEMANTIC_EPISODE_READY episode={index:06d}", flush=True)
    print_story(story, "APPROVED")
    print(
        f"  Population: {participants['cosmos_count']} Cosmos + "
        f"{participants['random_count']} random = {participants['total_count']}",
        flush=True,
    )
    for actor in participants["participants"]:
        print(f"  P{actor['actor_id']}: {actor['role']} - {actor['purpose']}", flush=True)
    for scene in plan["scenes"]:
        print(f"  Scene {scene['scene_id']}: {scene['description']}", flush=True)
        for actor in scene["actor_plans"]:
            stops = [actor["start_cell"]]
            for step in actor["subgoals"]:
                action = step["action"]
                if action == "Queue":
                    action += f" {step['queue_id']}; service {step.get('service_action', 'Idle')}"
                stops.append(f"{step['cell_id']} ({action}, {step['duration_s']:g}s)")
            stops.append(actor["goal_cell"])
            print(f"    P{actor['actor_id']}: {' -> '.join(stops)}", flush=True)
