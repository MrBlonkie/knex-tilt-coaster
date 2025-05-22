# knex-tilt-coaster

TODO:
- klemmen voor kabels (voeding)
- zekering? vragen aan MRG
- sensor states (block zoning)
- bottomLifthillSensor implementeren
- hallsensor achter drop wordt niet getriggerd
- flow programma
- clean up code with .h files

PROGRAM FEATURES:
- ping pong bij opstarten met alle ESP's
- verschillende modi


<img src="/pictures/tiltdrop_WIP.jpg">

aanpak:

per segment functies maken per actie
segment automatisch laten werken

master/slave geimplementeerd, eerst 1 master 1 slave, later uitbreiden
functies die al geprogrammeerd waren (en mogelijks combineren) en in commandos steken