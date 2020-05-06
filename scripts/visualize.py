import os
import csv
import multiprocessing
import numpy as np
from matplotlib import pyplot as plt

show_plt = True
figure_dpi = 128
figure_size = (7, 5)

results_dir = "results"

serial_file = "serial.csv"
omp_file = "omp_?.csv"
cuda_file = "cuda.csv"

def saturated_average(lst):
    # XXX: no saturation test here, eyeball data indicates after 57 iterations,
    #      frame times converge at uniform moving speed
    converge_idx = 57
    return sum(lst[converge_idx:]) / len(lst[converge_idx:])

# Visulaize Serial
def visualize_serial():
    avg_ftime = -1
    with open(serial_file) as f:
        times = []
        npts = []
        frametimes = []

        reader = csv.reader(f)
        _ = next(reader)
        for row in reader:
            times.append(int(row[0]))
            npts.append(int(row[1]))
            frametimes.append(int(row[2]))
        
        fig = plt.figure(dpi=figure_dpi, figsize=figure_size)
        ax = fig.add_subplot(1,1,1)
        plt.plot(npts, frametimes, 'o')
        plt.xlabel("Points Processed")
        plt.ylabel("Frame Time (us)")

        z = np.polyfit(npts, frametimes, 1)
        p = np.poly1d(z)
        plt.plot(npts, p(npts), "r--")
        plt.suptitle("Frame Time w.r.t. Points Processed", fontweight='bold')
        ax.set_title("Serial Implementation, Linear Fit = " + str(z))

        # ax.yaxis.set_ticks(np.arange(1, len(frametimes), 5))
        plt.savefig(os.path.join(results_dir, "serial_ftime_npoints.png"))

        avg_ftime = saturated_average(frametimes)
    return avg_ftime

# Visualize OpenMP
def visualize_omp():
    core_count = multiprocessing.cpu_count()
    times = {}
    npts = {}
    frametimes = {}
    average_frametimes = {}
    for core_idx in range(core_count):
        n_cores = core_idx + 1

        times[n_cores] = []
        npts[n_cores] = []
        frametimes[n_cores] = []

        f = open(omp_file.replace("?", str(n_cores)))
        reader = csv.reader(f)
        _ = next(reader)
        for row in reader:
            times[n_cores].append(int(row[0]))
            npts[n_cores].append(int(row[1]))
            frametimes[n_cores].append(int(row[2]))
        f.close()

        average_frametimes[n_cores] = saturated_average(frametimes[n_cores])

        # Plot frame time w.r.t. points processed
        if (n_cores % 2):
            fig = plt.figure(dpi=figure_dpi, figsize=figure_size)
            ax = fig.add_subplot(1,1,1)
            x = npts[n_cores][1:]
            y = frametimes[n_cores][1:]
            plt.plot(x, y, 'o')
            plt.xlabel("Points Processed")
            plt.ylabel("Frame Time (us)")

            z = np.polyfit(x, y, 1)
            p = np.poly1d(z)
            plt.plot(x, p(x), "r--")
            plt.suptitle("Frame Time w.r.t. Points Processed", fontweight='bold')
            ax.set_title("OpenMP #Core=" + str(n_cores) + ", Linear Fit = " + str(z))

            plt.savefig(os.path.join(results_dir, "omp_ftime_npoints?.png".replace("?", str(n_cores))))
    
    # Plot average frame time w.r.t. #threads
    fig = plt.figure(dpi=figure_dpi, figsize=figure_size)
    ax = fig.add_subplot(1,1,1)
    x = list(average_frametimes.keys())
    y = list(average_frametimes.values())
    plt.plot(x, y, 'o-')
    plt.xlabel("Core Count")
    plt.ylabel("Average Saturated Frame Time (us)")

    plt.suptitle("Average Frame Time w.r.t. Core Count", fontweight='bold')
    ax.set_title("OpenMP Implementation")

    # ax.yaxis.set_ticks(np.arange(1, len(frametimes), 5))
    plt.savefig(os.path.join(results_dir, "omp_core_comparison.png"))

    return average_frametimes

def visualize_omp_trend():
    avg_ftime = -1

    n_cores = multiprocessing.cpu_count()
    with open(omp_file.replace("?", str(2))) as f:
        times = []
        npts = []
        frametimes = []

        reader = csv.reader(f)
        _ = next(reader)
        _ = next(reader) # Skip first outlier
        for row in reader:
            times.append(int(row[0]))
            npts.append(int(row[1]))
            frametimes.append(int(row[2]))
        
        fig = plt.figure(dpi=figure_dpi, figsize=figure_size)
        ax = fig.add_subplot(1,1,1)
        plt.plot(npts, frametimes, 'o')
        plt.xlabel("Points Processed")
        plt.ylabel("Frame Time (us)")

        z = np.polyfit(npts, frametimes, 1)
        p = np.poly1d(z)
        plt.plot(npts, p(npts), "r--")
        plt.suptitle("Frame Time w.r.t. Points Processed", fontweight='bold')
        ax.set_title("OpenMP Implementation, Linear Fit = " + str(z))

        # ax.yaxis.set_ticks(np.arange(1, len(frametimes), 5))
        plt.savefig(os.path.join(results_dir, "omp_ftime_npoints.png"))

        avg_ftime = saturated_average(frametimes)
    return avg_ftime

# Visualize CUDA
def visualize_cuda():
    avg_ftime = -1
    with open(cuda_file) as f:
        times = []
        npts = []
        frametimes = []

        reader = csv.reader(f)
        _ = next(reader)
        for row in reader:
            times.append(int(row[0]))
            npts.append(int(row[1]))
            frametimes.append(int(row[2]))
        
        fig = plt.figure(dpi=figure_dpi, figsize=figure_size)
        ax = fig.add_subplot(1,1,1)
        plt.plot(npts, frametimes, 'o')
        plt.xlabel("Points Processed")
        plt.ylabel("Frame Time (us)")

        z = np.polyfit(npts, frametimes, 1)
        p = np.poly1d(z)
        plt.plot(npts, p(npts), "r--")
        plt.suptitle("Frame Time w.r.t. Points Processed", fontweight='bold')
        ax.set_title("CUDA Implementation, Linear Fit = " + str(z))

        # ax.yaxis.set_ticks(np.arange(1, len(frametimes), 5))
        plt.savefig(os.path.join(results_dir, "cuda_ftime_npoints.png"))

        avg_ftime = saturated_average(frametimes)
    return avg_ftime

if __name__ == "__main__":
    core_count = multiprocessing.cpu_count()

    if not os.path.exists(results_dir):
        os.makedirs(results_dir)

    avg_ftime_serial = visualize_serial()
    avg_ftime_omp    = visualize_omp()
    avg_ftime_cuda   = visualize_cuda()

    # visualize_omp_trend()

    print("Average Frame Time for [Serial] Version: ", avg_ftime_serial)
    print("Average Frame Time for [OpenMP] Version: ", avg_ftime_omp)
    print("Average Frame Time for [ CUDA ] Version: ", avg_ftime_cuda)

    # Compare average time between serial and OpenMP version
    fig = plt.figure(dpi=figure_dpi, figsize=figure_size)
    ax = fig.add_subplot(1,1,1)
    xs = ["Serial"]
    ys = [avg_ftime_serial]
    colors = ["green"]
    
    for i in range(1, core_count + 1):
        xs.append(str(i))
        ys.append(avg_ftime_omp[i])
        colors.append("blue")

    xs.append("GPU")
    ys.append(avg_ftime_cuda)
    colors.append("red")

    ax.bar(xs, ys, color=colors)
    plt.suptitle("Average Frame Time Performance Comparison", fontweight='bold')
    ax.set_title("Serial, OpenMP with different #cores, CUDA")
    
    plt.xlabel("Implementation")
    plt.ylabel("Frame Time (us) (Lower is better)")
    
    legend_key = {"Serial": "green", "OpenMP": "blue", "CUDA": "red"}
    labels  = list(legend_key.keys())
    handles = [plt.Rectangle((0,0),1,1, color=legend_key[label]) for label in labels]
    plt.legend(handles, labels)

    plt.savefig(os.path.join(results_dir, "impl_ftime_comparison.png"))
    if show_plt:
        plt.show()