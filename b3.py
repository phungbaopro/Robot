import numpy as np

def af(x):
    y = np.tanh(x)
    return y

def vidu_nn(X, W, V):
    Net = W.T @ X
    y_h = af(Net)
    output = V.T @ y_h
    return output

def cost_function(W, V):
    X1 = [0.3, 0.35, 0.4, 0.8, 0.9, 1.0, 1.2, 1.6, 2.0]
    X2 = [0.3, 0.4, 0.5, 0.75, 0.7, 0.8, 0.4, 0.5, 0.5]
    Y = [1, 1, 1, 2, 2, 2, 4, 4, 4]
    
    X = np.array([X1, X2])
    Y = np.array([Y])
    
    y_hat = vidu_nn(X, W, V)
    e = Y - y_hat
    J = np.sum(e**2)
    return J

pop_size = 100
min_max = [-1, 1]
npar = 2
w = 0.9
c1 = 1
c2 = 1
max_iteration = 10000

Pbest_position = np.zeros((pop_size, npar))
Gbest_position = np.zeros((1, npar))
Pbest_fitness = np.ones(pop_size) * 999999
Gbest_fitness = 999999

pop = np.random.uniform(min_max[0], min_max[1], (pop_size, npar))
V = pop * 0

for i in range(max_iteration):
    # Đánh giá
    for index, individual in enumerate(pop):
        W = individual[:20].reshape(2, 10)
        V1 = individual[20:].reshape(10, 1)
        
        J = cost_function(W, V1)
        
        if J < Pbest_fitness[index]:
            Pbest_fitness[index] = J
            Pbest_position[index] = individual
        
        if J < Gbest_fitness:
            Gbest_fitness = J
            Gbest_position = individual
    
    # Cập nhật
    V = w * V + c1 * np.random.rand() * (Pbest_position - pop) + c2 * np.random.rand() * (Gbest_position - pop)
    pop = pop + V
    
    print(f"Iteration: {i}, Best fitness: {Gbest_fitness}")

print(Gbest_position)

W = Gbest_position[:20].reshape(2, 10)
V1 = Gbest_position[20:].reshape(10, 1)

X1 = [0.3, 0.35, 0.4, 0.8, 0.9, 1.0, 1.2, 1.6, 2.0]
X2 = [0.3, 0.4, 0.5, 0.75, 0.7, 0.8, 0.4, 0.5, 0.5]

X = np.array([X1, X2])

YYY = vidu_nn(X, W, V1)
print(YYY)
