import glob, random, os
while True:
    problems = [p for p in glob.glob('*') if ('run.py' != p and 'tmp-' not in p)]
    if len(problems) == 0:
        print(f'\n\tAll done!')
        exit(0)
    problem = random.choice(problems)

    with open(problem, 'r') as f:
        cmdline = f.read()
    print(f'\n\tRunning: {cmdline}')

    os.system(f'rm {problem}')

    os.system(cmdline)
