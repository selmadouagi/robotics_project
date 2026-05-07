# Analyse des fonctions répétitives et optimisations possibles

## 1. **Fonctions d'obstacles TRÈS répétitives** ⭐⭐⭐
Les trois fonctions `front_obstacle()`, `left_obstacle()`, `right_obstacle()` font la même chose:
- Créer un tableau d'indices de capteurs
- Boucler dessus pour obtenir les valeurs
- Retourner le maximum (ou médiane pour left/right)

**Opportunité**: Créer une fonction générique `get_sensor_value(sensor_ids[], size, use_median)`

```cpp
// Actuel - RÉPÉTITIF
double MyRobot::front_obstacle()      // indices: 0,1,14,15 → max
double MyRobot::left_obstacle()       // indices: 4,5,6     → médiane
double MyRobot::right_obstacle()      // indices: 9,10,11   → médiane
```

---

## 2. **Détection verte - fonction dupliquée** ⭐⭐
`detect_green_victim()` et `victim_position_in_image()` ont du code identique:
- Les deux parcourent les pixels
- Les deux cherchent le vert avec les mêmes conditions
- Les deux peuvent être fusionnées

**Opportunité**: Une seule fonction qui retourne ratio + position

```cpp
// Actuel
bool detect_green_victim()                        // retourne bool
void victim_position_in_image(ratio, center_x)   // retourne position
```

---

## 3. **Probes droite/gauche - TRÈS similaires** ⭐⭐⭐
`ID_PROBE_RIGHT_CENTER` et `ID_PROBE_LEFT_CENTER` font exactement la même logique:
- Calculer la distance parcourue
- Vérifier si on a touché quelque chose
- Reculer si oui

**Opportunité**: Créer une fonction helper `probe_forward(side_name)` appelée depuis les deux cas

---

## 4. **Scans droite/gauche - PRESQUE identiques** ⭐⭐⭐
`ID_DRIVE_TO_RIGHT_WALL` et `ID_DRIVE_TO_LEFT_WALL`:
- Même boucle de check front_obstacle
- Même hit counter logic
- Même gestion du heading drift

**Opportunité**: Fonction helper `scan_wall(side, target_heading)` 

---

## 5. **Backup après probe - Code dupliqué** ⭐⭐
`ID_BACKUP_RIGHT_CENTER` et `ID_BACKUP_LEFT_CENTER` sont identiques

**Opportunité**: Une fonction `backup_distance(target_distance)`

---

## 6. **Retours vers la ligne - Duplicatés** ⭐⭐
`ID_RETURN_FROM_RIGHT` et `ID_RETURN_FROM_LEFT` - même logique de retour

**Opportunité**: Fonction `return_to_anchor()`

---

## Résumé des améliorations possibles:

| Fonction | Type | Gain |
|----------|------|------|
| Obstacles (front/left/right) | Générique | **-50 lignes** |
| Détection verte | Fusionner | **-30 lignes** |
| Probes right/left | Helper | **-60 lignes** |
| Scans right/left | Helper | **-80 lignes** |
| Backups right/left | Helper | **-25 lignes** |
| Returns right/left | Helper | **-20 lignes** |
| **TOTAL** | | **~265 lignes** |
