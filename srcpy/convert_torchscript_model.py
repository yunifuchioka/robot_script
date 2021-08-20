import numpy
import torch

from model import ActorCriticNet

model = ActorCriticNet(31, 8, [128, 128])
model.load_state_dict(torch.load("../../../../models/iter9900.pt", map_location=torch.device('cpu')))

example_input = torch.rand(1,31)

model_script = torch.jit.trace(model, example_input)

model_script.save("../../../../models/iter9900_script.pt")