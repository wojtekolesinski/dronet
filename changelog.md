# Changelog:
- addded a random value to the initial speed and the initial battery.
- added a partial implementation of the q-fanet
- Continued q-fanet by adding both the QMR and q-learning+ module.
- fixed the reward function for every case: 
    - -100 if the node is a minimum (in relay_selection) or if the packet expires
    - +100 of the packet arrives to the destination
    - +50 to the previous drone if the packet can be successfully passed to the next drone.
- fixed bug (it was ```if np.all(distances > 0):``` instead of ```if np.all(distances < 0):```)
