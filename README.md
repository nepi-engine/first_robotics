# First Robotics

This readme goes over the workflow you will have working with this repo.

You clone the repo once to get a local copy on your own machine. From then on, you edit code locally, deploy it to the NEPI system to test it, and push it back to GitHub so the rest of the team picks it up.

The four sections below cover each step:

- **Cloning** — Creating a local working copy of the repo on your computer
- **Deploying** — Syncing your local changes onto your NEPI device
- **Pulling** — Merging your teammates' changes into your local repo
- **Pushing** — Committing your local changes and publishing them to the shared repo

Each section lists the commands to run and the directory to run them from. Deploying additionally requires network access to the NEPI system and an SSH key already configured on your machine.

## Cloning

The first step to working in the first robotics repo is to clone it to your computer so you can make edits.

First, open your terminal:

- Windows: open "Git Bash" (installed with Git) from the Start menu

- Mac: open the "Terminal" app (Spotlight search → "Terminal")

- Either OS: you can also use the built-in terminal in VS Code (View → Terminal)

Then check to make sure you are connected to the internet:

```
ping -c 1 google.com
```

Then run this command to go to your home directory:

```
cd ~/
```

Then clone the repo:

```
git clone git@github.com:nepi-engine/first_robotics.git
```

After running this command, there is a local copy of the first_robotics repo on your computer that you can edit.

## Deploying

While you are working on your app, it's important to deploy your changes so you can see them running on the NEPI system.

There are two ways to deploy: deploying just the app you're working on, or deploying the entire repo.

### Deploying a single app

This is the fastest option, and the one you'll use most while actively working on one app.

First, make sure you're in the first_robotics folder (run this from your home directory):

```
cd first_robotics/
```

Then move into the folder for the app you're working on. For example:

```
cd nepi_app_obstacles/
```

Then run:

```
./deploy_app.sh
```

This copies your app's code to the NEPI build location and pushes the updated scripts to the running NEPI system, so your changes take effect right away.

**NOTE:** Every app has its own `deploy_app.sh` (nepi_app_auto_move, nepi_app_obstacles, nepi_app_stereo_cam, nepi_app_wpilib_if) — you must run it from inside that app's own folder.

### Deploying the entire repo

Use this option when you want to sync everything at once. For example, after pulling changes your teammates made to other apps.

First, make sure you're in the first_robotics folder (run this from your home directory):

```
cd first_robotics/
```

Then run:

```
./deploy_repo_complete.sh
```

This deploys the NEPI engine overrides, all of the apps (nepi_app_auto_move, nepi_app_obstacles, nepi_app_stereo_cam, nepi_app_wpilib_if), and the test data to the NEPI system in one go.

**NOTE:** Deploying the whole repo takes longer than deploying a single app, since it syncs everything.

## Pulling

While you are working on your local repo, it's important to keep it up to date with the latest changes your other team members have added.

First, make sure you're in the first_robotics folder (run this from your home directory):

```
cd first_robotics/
```

Then to pull changes, run the following command:

```
git pull
```

This will bring your local repo up to date with all the changes.

**NOTE:** Pull before you start working each session, so you're editing the latest version of the code.

## Pushing

Once you have made changes to your local repo that you're happy with, you can push your changes to the repo so everyone else can see them.

First, make sure you're in the first_robotics folder (run this from your home directory):


```
cd first_robotics/
```

Then run:

```
pushn
```

**NOTE:** pushn is a custom command our team has already set up on your system — it's not a standard Git command. It stages, commits, and pushes your changes for you in one step.

### If someone else pushed changes first

If you're pushing your changes and someone else has updated the repo since your last pull, you'll see an error that says $\color{red}\textsf{Push Failed}$.

If you get this error, run:

```
git pull
```

This will bring up a window with blue text showing that the branches have merged:

Exit this window by hitting Ctrl+X. This will return you to your terminal. Then run:

```
pushn
```

This will finish pushing your changes to the repo and you will see a message in your terminal saying $\color{green}\textsf{Push Successfully}$.