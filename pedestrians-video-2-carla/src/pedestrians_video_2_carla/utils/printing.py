def print_metrics(results, header):
    """
    Prints the human-readable metrics.
    """

    print('\n------------------------------------------------------')
    print(header)
    print('------------------------------------------------------')
    for k, v in results.items():
        print(f'{k}: {v}')
    print('------------------------------------------------------\n')
