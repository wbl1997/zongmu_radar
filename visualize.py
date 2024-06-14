import argparse
import numpy as np
import matplotlib.pyplot as plt

import sys

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('datafile', type=str, default='data.csv.npy', help='data file')

    args = parser.parse_args()

    data = np.load(args.datafile)
    print(data.shape)
    print('first {}'.format(data[0, :]))
    print('last {}'.format(data[-1, :]))

    # visualize
    fig, axs = plt.subplots(2, 2)

    axs3d = []
    for i in range(4):
        mfig = plt.figure()
        ax = mfig.add_subplot(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        axs3d.append(ax)

    frames = [[], [], [], []]
    npframes = [None, None, None, None]
    last_sensor = -1
    count = 1
    for i in range(data.shape[0]):
        if data[i, 4] == 1 and last_sensor == 4:
            for j in range(4):
                m = j // 2
                n = j % 2
                if len(frames[j]) == 0:
                    continue
                npframes[j] = np.array(frames[j])
                axs[m, n].cla()
                axs[m, n].plot(npframes[j][:, 0], npframes[j][:, 1], 'b.')
                axs[m, n].set_title("({}, {})".format(m, n))
                axs3d[j].cla()
                axs3d[j].scatter(npframes[j][:, 0], npframes[j][:, 1], npframes[j][:, 2], 'b.')
                axs3d[j].set_title("{}".format(j))
            plt.pause(0.1)

            frames = [[], [], [], []]
            count += 1

        r, v, ad, ed = data[i, 7:11]
        a = ad * np.pi / 180.0
        e = ed * np.pi / 180.0
        x = r * np.cos(e) * np.cos(a)
        y = r * np.cos(e) * np.sin(a)
        z = r * np.sin(e)
        frames[int(data[i, 4]) - 1].append([x, y, z])
        last_sensor = data[i, 4]
    print('combined frame count {}'.format(count))

if __name__ == '__main__':
    main()
