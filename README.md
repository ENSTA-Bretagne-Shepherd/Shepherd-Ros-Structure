# Shepherd-Ros-Structure

__Version ROS__: Indigo

## Demo

Pour afficher un premier exemple de demonstration (simulation voilier uniquement):

```bash
$ catkin_make
$ source devel/setup.bash
$ roslaunch shepherd_simu demo1.launch
```

Pour visionner les noeuds et changer les constantes d'environnement (vent, centres des triangles).
Dans un __nouveau terminal__:

```bash
$ source devel/setup.bash
$ rqt --perspective-file preferences/rqt/Observation.perspective
```

## Informations

### Structure des noeuds

L'architecture des noeuds de ce repository est disponible [ici](https://github.com/ENSTA-Bretagne-Shepherd/Shepherd-Ros-Structure/blob/master/structure.pdf)

### Utilisation

```bash
catkin_make
source devel/setup.bash
```
  	
### Comment contribuer

Il y a 4 packages différents:
 - __shepherd_disp__ : ici se trouvent les noeuds de _display_ (unity, openGL, morse ...)
 - __shepherd_simu__ : ici se trouvent les noeuds de _simulation_ (voiliers, bouées ...)
 - __shepherd_loc__  : ici se trouvent les noeuds de _localisation_ (méthodes par intervalles ...)
 - __shepherd_reg__  : ici se trouvent les noeuds de _régulation_ (suivi de triangles, flocking ...)

Pour contribuer:

1. Créez une branche pour la fonctionnalité sur laquelle vous travaillez.
2. Mettez-y (dans le package correspondant) tout votre travail.
3. __NE MERGEZ PAS SUR LA BRANCHE MASTER__. Nous ferons les merges ensemble, lorsque nous serons sur que ca marche


__NB__:
* convention pour le nom de la branche: `<nom_du_package>_<fonctionnalité>`  
		_par exemple: si vous travaillez sur un noeud de localisation concernant les bouées nommez la branche:_  `loc_buoy`
