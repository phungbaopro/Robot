# import numpy as np

# #Khai tao
# pop_size = 10
# min_max = [-5, 55]
# npar = 2
# w = 0.9
# c1 = 1
# c2 = 1
# max_iteration = 1000

# Pbest_position = np.zeros((pop_size, npar))
# Gbest_position = np.zeros((1, npar))
# Pbest_fitness = np.ones(pop_size) * 9999999
# Gbest_fitness = 9999999

# # print(Pbest_position)


# pop = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
# print(pop)
# v = pop * 0

# #Danh gia
# def loss_function(x, y):
#     output = 2 * x**2 - 1.05 * x**4 + x**6 / 6 + x * y + y**2
#     return output

# for i in range(max_iteration):
#     # danh gia
#     for index, invidual in enumerate(pop):
#         J = loss_function(invidual[0], invidual[1])
#         # print(index, invidual, J)
#         if (J < Pbest_fitness[index]):
#             Pbest_fitness[index] = J
#             Pbest_position[index] = invidual
#         if (J < Gbest_fitness):
#             Gbest_fitness = J
#             Gbest_position = invidual

#     # update
#     v = w * v + c1 * np.random.rand() * (Pbest_position - pop) + c2 * np.random.rand() * (Gbest_position - pop)
#     pop = pop + v
#     # print(pop)
#     print(f"Iteration: {i}, Best fitness: {Gbest_fitness}")

# print(Gbest_position)

import numpy as np

# Khai tao
pop_size = 10
min_max = [-10, 10]
npar = 2
w = 0.9
c1 = 1
c2 = 1
max_iteration = 1000

Pbest_position = np.zeros((pop_size, npar))
Gbest_position = np.zeros((1, npar))
Pbest_fitness = np.ones(pop_size) * 9999999
Gbest_fitness = 9999999

pop = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
print(pop)
v = pop * 0

# Danh gia
def loss_function(x, y):
    output = -0.0001 * (np.abs(np.sin(x) * np.sin(y) * np.exp(np.abs(100 - (np.sqrt(x**2 + y**2) / np.pi)))) + 1) ** 0.1
    return output

for i in range(max_iteration):
    # danh gia
    for index, invidual in enumerate(pop):
        J = loss_function(invidual[0], invidual[1])
        if (J < Pbest_fitness[index]):
            Pbest_fitness[index] = J
            Pbest_position[index] = invidual
        if (J < Gbest_fitness):
            Gbest_fitness = J
            Gbest_position = invidual

    # update
    v = w * v + c1 * np.random.rand() * (Pbest_position - pop) + c2 * np.random.rand() * (Gbest_position - pop)
    pop = pop + v
    print(f"Iteration: {i}, Best fitness: {Gbest_fitness}")

print(Gbest_position)

