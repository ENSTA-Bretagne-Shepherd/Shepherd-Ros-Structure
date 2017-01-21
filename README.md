# Shepherd-Ros-Structure

__Version ROS__: Indigo

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
3. Nous ferons les merges ensemble
