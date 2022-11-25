import numpy as np
import matplotlib.pyplot as plt


def return_state_vector(ddp):
    """Creates an array containing the states along the horizon

    Arguments:
        dpp -- Crocoddyl object containing all solver related data
    """
    raw_states = np.array(ddp.xs)

    nq = int((len(raw_states[0]) - 7) / 2 + 3)

    states = dict(
        base_position=raw_states[:, 0:3],
        base_linear_velocity=raw_states[:, nq : (nq + 3)],
        base_orientation=raw_states[:, 3:7],
        base_angular_velocity=raw_states[:, (nq + 3) : (nq + 6)],
        joints_position=raw_states[:, 7:nq],
        joints_velocity=raw_states[:, (nq + 6) :],
    )

    return states


def return_command_vector(ddp):
    """Creates an array containing the commands along the horizon

    Arguments:
        dpp -- Crocoddyl object containing all solver related data
    """
    commands = np.array(ddp.us)

    return commands


def return_cost_vectors(ddp, weighted=False, integrated=False):
    """
    Creates a dictionary with the costs along the horizon from the ddp object and returns it
    """
    costs = {}
    for i in range(ddp.problem.T):
        for cost_tag in list(
            ddp.problem.runningModels[i].differential.costs.costs.todict().keys()
        ):
            if cost_tag not in costs:
                costs.update({cost_tag: np.nan * np.ones(ddp.problem.T + 1)})
            try:
                costs[cost_tag][i] = (
                    ddp.problem.runningDatas[i].differential.costs.costs[cost_tag].cost
                )
            except:
                costs[cost_tag][i] = np.mean(
                    [
                        diff.costs.costs[cost_tag].cost
                        for diff in ddp.problem.runningDatas[i].differential
                    ]
                )
            if weighted:
                costs[cost_tag][i] *= (
                    ddp.problem.runningModels[i]
                    .differential.costs.costs.todict()[cost_tag]
                    .weight
                )
            if integrated:
                costs[cost_tag][i] *= ddp.problem.runningModels[i].dt

    for cost_tag in list(
        ddp.problem.terminalModel.differential.costs.costs.todict().keys()
    ):
        if cost_tag not in costs:
            costs.update({cost_tag: np.nan * np.ones(ddp.problem.T + 1)})
        try:
            costs[cost_tag][-1] = ddp.problem.terminalData.differential.costs.costs[
                cost_tag
            ].cost
        except:
            costs[cost_tag][-1] = np.mean(
                [
                    diff.costs.costs[cost_tag].cost
                    for diff in ddp.problem.terminalData.differential
                ]
            )
        # if weighted:
        #     costs[cost_tag][-1] *= ddp.problem.terminalModel.differential.costs.costs.todict()[cost_tag].weight

    return costs


def return_weights(ddp):
    weights = {}
    for i in range(ddp.problem.T):
        for cost_tag in list(
            ddp.problem.runningModels[i].differential.costs.costs.todict().keys()
        ):
            if cost_tag not in weights:
                weights.update({cost_tag: np.nan * np.ones(ddp.problem.T + 1)})
            weights[cost_tag][i] = (
                ddp.problem.runningModels[i]
                .differential.costs.costs.todict()[cost_tag]
                .weight
            )

    for cost_tag in list(
        ddp.problem.terminalModel.differential.costs.costs.todict().keys()
    ):
        if cost_tag not in weights:
            weights.update({cost_tag: np.nan * np.ones(ddp.problem.T + 1)})
        weights[cost_tag][
            -1
        ] = ddp.problem.terminalModel.differential.costs.costs.todict()[cost_tag].weight

    return weights


def return_time_vector(ddp, t0=0):
    """
    Returns a vector with the time evolution related to a ddp problem,
    useful when plotting and dt changes from node to node
    """
    time = np.zeros(ddp.problem.T + 1)
    if t0 != 0:
        time[0] = t0
    for i in range(1, ddp.problem.T + 1):
        time[i] = time[i - 1] + ddp.problem.runningModels[i - 1].dt
    return time


def plot_costs_from_dic(dic):
    """
    Plots dictionary tags
    """
    fig, ax_running = plt.subplots()
    ax_terminal = ax_running.twinx()
    runningCosts = []
    terminalCosts = []
    if "time" in dic:
        time = dic["time"]
    else:
        time = np.arange(0, len(dic[list(dic.keys())[0]]))
    for tag in list(dic.keys()):
        if np.sum(np.isnan(dic[tag])) == 0:
            ax_running.plot(dic[tag])
            runningCosts.append(tag)
        else:
            ax_terminal.scatter(time, dic[tag])
            terminalCosts.append(tag)
    ax_running.legend(runningCosts)
    ax_terminal.legend(terminalCosts)
    plt.show()


def plot_state_from_dic(dic):
    fig = plt.figure()

    for i in range(len(list(dic.keys()))):
        key = list(dic.keys())[i]
        ax = fig.add_subplot(3, 2, i + 1)
        ax.plot(dic[key])
        ax.set_title(key)


def plot_command(commands):
    fig = plt.figure()
    plt.plot(commands)
    plt.title("Commands")
