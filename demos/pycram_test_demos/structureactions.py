# perception_module.py
class PerceptionModule:
    def perceive(self, sensory_data):
        # Process sensory data and convert to internal representation
        perception = {"type": "object", "location": sensory_data.get("location")}
        return perception

# knowledge_base.py
class KnowledgeBase:
    def __init__(self):
        self.facts = {}

    def update_fact(self, key, value):
        self.facts[key] = value

    def get_fact(self, key):
        return self.facts.get(key)

# planning_module.py
class PlanningModule:
    def __init__(self, knowledge_base):
        self.knowledge_base = knowledge_base

    def plan_action(self, goal):
        if goal == "move":
            return [("move", {"direction": "north", "distance": 10})]
        elif goal == "cut":
            return [("cut", {"object": "bread", "tool": "knife"})]
        elif goal == "pour":
            return [("pour", {"source": "bottle", "target": "cup"})]
        else:
            raise ValueError(f"Unknown goal: {goal}")

# execution_module.py
from abc import ABC, abstractmethod

class Action(ABC):
    @abstractmethod
    def execute(self):
        pass

class MoveAction(Action):
    def __init__(self, parameters):
        self.parameters = parameters

    def execute(self):
        print(f"Executing move action with parameters: {self.parameters}")

class CutAction(Action):
    def __init__(self, parameters):
        self.parameters = parameters

    def execute(self):
        print(f"Executing cut action with parameters: {self.parameters}")

class PourAction(Action):
    def __init__(self, parameters):
        self.parameters = parameters

    def execute(self):
        print(f"Executing pour action with parameters: {self.parameters}")

class ActionFactory:
    @staticmethod
    def create_action(action_name, parameters):
        if action_name == 'move':
            return MoveAction(parameters)
        elif action_name == 'cut':
            return CutAction(parameters)
        elif action_name == 'pour':
            return PourAction(parameters)
        else:
            raise ValueError(f"Unknown action: {action_name}")

class ExecutionModule:
    def execute_plan(self, plan):
        for action_name, parameters in plan:
            action = ActionFactory.create_action(action_name, parameters)
            action.execute()

# learning_module.py
class LearningModule:
    def __init__(self, knowledge_base):
        self.knowledge_base = knowledge_base

    def learn_from_outcome(self, action, outcome):
        self.knowledge_base.update_fact(action, outcome)

# reasoning_module.py
class ReasoningModule:
    def __init__(self, knowledge_base):
        self.knowledge_base = knowledge_base

    def reason_about_goal(self, goal):
        if goal == "move":
            location = self.knowledge_base.get_fact("location")
            if location:
                return f"Moving from {location}"
        return f"Executing goal: {goal}"

# main.py
def main():
    sensory_data = {"location": "start"}
    goal = "cut"

    knowledge_base = KnowledgeBase()
    perception_module = PerceptionModule()
    planning_module = PlanningModule(knowledge_base)
    execution_module = ExecutionModule()
    learning_module = LearningModule(knowledge_base)
    reasoning_module = ReasoningModule(knowledge_base)

    perception = perception_module.perceive(sensory_data)
    knowledge_base.update_fact("location", perception["location"])

    reasoning = reasoning_module.reason_about_goal(goal)
    print(reasoning)

    plan = planning_module.plan_action(goal)
    print(f"Plan: {plan}")

    execution_module.execute_plan(plan)

    learning_module.learn_from_outcome(goal, "success")

if __name__ == "__main__":
    main()
