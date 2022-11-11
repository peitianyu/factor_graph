import matplotlib.pyplot as plt

# pose: 2.64216 1.61589 0.428317
def show_data(file_name):
    with open(file_name, 'r') as f:
        line = f.readline()
        x = []
        y = []
        while line:
            x.append(float(line.split()[1]))
            y.append(float(line.split()[2]))
            line = f.readline()
    plt.plot(x, y)
    plt.show()
    pass

def main():
    show_data('../log/original.txt')
    show_data('../log/optimized.txt')
    pass

if __name__ == '__main__':
    main()
    pass