# pendulum-core

![pendulum](pendulum-v2.jpg)

Support logiciel du démonstrateur "Pendule Inversé".

Il se décompose en deux parties :

- la couche bas niveau, assurant deux rôles principaux :
    - interfaçage avec le hardware à l'aide de la librairie `pigpio`
    - conditionnement par filtrage des signaux des encodeurs de position et
      d'angle
- l'application de démonstration, permettant d'interagir avec le pendule :
    - en contrôle 100% manuel
    - en activant l'asservissement LQR maintenant l'équilibre instable lorsque
      le pendule est au voisinage de la position
    - en activant le contrôle 100% automatique, qui amène dans un premier temps
      le pendule au voisinage de l'équilibre, puis passe en mode stabilisation
      automatique LQR lorsque les conditions nécessaires sont remplies.

Les communications entre les deux couches sont assurées par [**ZeroMQ**](https://zeromq.org/).

## Crédits

Ce projet est basé sur les travaux menés par Franco FUSCO dans le cadre de
la préparation de sa thèse. Le repository original est disponible
[ici](https://github.com/francofusco/pendule_pi).
