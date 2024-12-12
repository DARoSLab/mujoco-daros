import torch
input = torch.tensor([2, 3, 4, 5], dtype = float)
input.requires_grad = True

def f(x):
    return x**3

output = f(input)
# output.backward()
output.backward(torch.tensor([1, 1, 1, 1], dtype = float))
print(input.grad)
