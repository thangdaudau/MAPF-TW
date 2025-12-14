from core.wrapper import ILP, CBS, ICBS, GCBS
from core.wrapper import set_time_limit
import utils

if __name__ == '__main__':
    from argparse import ArgumentParser, RawTextHelpFormatter

    # --- Define verbose descriptions ---
    ALGO_CHOICES = ['ILP', 'CBS', 'ICBS', 'GCBS']
    ALGO_HELP_MSG = (
        "The Multi-Agent Path Finding (MAPF) algorithm to execute:\n"
        "  ILP: Integer Linear Programming (Guaranteed optimal, slow).\n"
        "  CBS: Conflict-Based Search (Optimal, faster than ILP).\n"
        "  ICBS: Improved CBS (Uses Meta-Agents, highly effective optimal solver).\n"
        "  GCBS: Greedy CBS (Suboptimal, extremely fast, no quality guarantee)."
    )

    # --- ArgumentParser Setup ---
    parser = ArgumentParser(
        description='A comprehensive tool for solving the Multi-Agent Path Finding (MAPF) problem with Time Windows (TW).',
        epilog='Example: python main.py GCBS 6x6x6',
        formatter_class=RawTextHelpFormatter # ESSENTIAL for preserving custom line breaks
    )

    # --- Argument: 'algo' ---
    parser.add_argument(
        'algo', 
        choices=ALGO_CHOICES, 
        help=ALGO_HELP_MSG
    )

    # --- Argument: 'res' ---
    parser.add_argument(
        'res', 
        help="The base name of the input.YAML data file located in the '/res' directory. "
            "Example: Enter '6x6x6' to load 'res/6x6x6.input.yaml'."
    )
    args = parser.parse_args()

    func_name = args.algo
    input_path = 'res/' + args.res + '.input.yaml'
    output_path = input_path.replace('input', 'output')

    V, E, S, n, m = utils.generate_input_from_yaml(input_path)
    T = 15

    set_time_limit(60)

    if func_name == 'ILP':
        result = ILP(V, E, S, T)
    elif func_name == 'CBS':
        result = CBS(n, m, E, S)
    elif func_name == 'ICBS':
        result = ICBS(n, m, E, S)
    elif func_name == 'GCBS':
        from mapf_solver import ConflictHeuristic
        result = GCBS(n, m, E, S)

    if result:
        cost, paths = result
        print(f'SOLUTION FOUND: Average_CS = {cost / len(paths)}')
        for e in paths:
            e += [e[-1], e[-1]]
            for i in range(len(e)):
                e[i] = (e[i] % m, e[i] // m)
        utils.save_output(output_path, paths)
    else:
        print('Time Up!')