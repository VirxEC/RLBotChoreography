# "The Sky" Choreography

This project can produce choreographed sequences of car motion in
Rocket League. It's useful for making cool synchronized performances.

## Welcome, Air Show Viewers!
![Airshow banner](airshow.png)

On Feb 20, 2020, Rocket Sledge released a youtube called the Rocket League Air Show
featuring performances created with this code. Three different teams worked in secret,
so the code is not merged together yet. For now we'll link you to the branches.

### Scripted
![Scripted](scripted.png)

* Direction and cinematography by ColemanA, bot sequences by tarehart
* https://github.com/tarehart/RLBotChoreography/tree/scripted

### \_Fool\_
![Fool](fool.png)

* Made by \_DaCoolOne\_
* https://github.com/DaCoolOne/HiveWriter

### Awakening
![Awakening](awakening.png)

* Made by Darxeal, Will, IamEld3st, and Jeroen11dijk
* https://github.com/Darxeal/Choreography

## Setup

![Bots doing the wave](wave.gif)

1. Run `setup.bat`

## Usage

### Simple

Just run `launch.bat`

### Advanced

In a command prompt, in this directory, run `python ChoreographyHive`

You can pass in an argument to specify the folder for bot appearances with `python ChoreographyHive --bot-folder=bots`
Other settings can be customised through the GUI.

- If you have a bunch of bots in your bot folder (e.g. maybe you grabbed https://github.com/RLBot/RLBotPack),
we will find all the bots there and use their appearances for the drones. There will be one drone spawned for each.

## Tutorial

Check out https://www.youtube.com/watch?v=F3OpOdUavfw

## Origins

This is based on 'hivemind' code shared by Viliam Vadocz.
Original code is here: https://github.com/ViliamVadocz/RLBot

The underlying framework is explained at http://www.rlbot.org/
