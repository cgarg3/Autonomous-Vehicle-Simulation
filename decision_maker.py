class DecisionMaker:
    def __init__(self, rule_based=True):
        self.rule_based = rule_based

    # checks whether the decision making should be rule based or machine learning based
    def make_decision(self, detected_objects, current_state):
        if self.rule_based:
            # Rule-based decision-making logic
            decision = self.rule_based_decision(detected_objects, current_state)
        else:
            # Machine learning-based decision-making logic
            decision = self.machine_learning_decision(detected_objects, current_state)

        return decision

    # method that contains a simple rule prioritizing stopping if pedestrains are detected
    def rule_based_decision(self, detected_objects, current_state):
        # Placeholder for rule-based decision-making logic
        # Example: If pedestrians detected, prioritize stopping
        if any(obj['class'] == 'person' for obj in detected_objects):
            decision = "Stop"
        else:
            decision = "Continue"

        return decision

    def machine_learning_decision(self, detected_objects, current_state):
        # Placeholder for machine learning-based decision-making logic
        # Example: Use a trained model to determine the optimal action
        # Replace this with the actual machine learning model inference
        decision = "ML Decision"
        
        return decision

# Example usage:
decision_maker = DecisionMaker(rule_based=True)
detected_objects = [{"class": "car", "position": (100, 200)}, {"class": "person", "position": (300, 150)}]
current_state = "Some State"

decision = decision_maker.make_decision(detected_objects, current_state)
print("Decision:", decision)
