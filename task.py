class Task:
    
    def __init__(self, type, goal=None, P=50, D=50, max_error=0.01, update_text=None):
        self.type = type
        self.goal = goal
        self.P = P
        self.D = D
        self.max_error = max_error
        self.update_text = update_text
    
    