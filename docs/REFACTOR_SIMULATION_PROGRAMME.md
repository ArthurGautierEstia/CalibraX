# Refactoring simulation mode Programme — état au 2026-06-12

Commit : `3212fbc` — Refactoring simulation programme : axes externes, affichage monde, calcul segment par segment.

## Ce qui a été fait

### Chaîne d'affichage (fix « trajectoire translatée avec le robot »)
- Les samples portent désormais `nominal_pose_world` ET `measured_pose_world`
  (`ProgramSimulationSample`, calculées dans `ProgramSimulator._build_sample` avec
  l'état d'axes externes simulé du moment).
- Le viewer accepte des points déjà en monde : `set_trajectory_path_segments(..., in_world=True)`
  et `set_trajectory_keypoints(..., in_world=True)` (Viewer3DWidget + Viewer3DController).
  Quand `in_world=True`, aucune re-transformation par la base robot → la trajectoire
  ne suit plus le robot sur le rail.
- `ProgramController` construit tous les segments depuis les poses monde et exclut les
  samples `EXTERNAL_AXIS` de l'affichage (axes positionnés, pas de tracé).
- Keypoints viewer : position monde = `T_world_robotBase @ pose_base` (état courant des axes).

### Fix playback bloqué (événements)
- `ProgramController._suppress_context_invalidation` : armé pendant `_apply_time_value`
  (animation axes externes) et le « Aller à » d'un mouvement d'axe ; `_on_external_chain_changed`
  et `_on_workpiece_changed` sortent immédiatement quand il est armé → plus d'invalidation
  de la simulation pendant la lecture.

### Séquence d'import / simulation
- Import = cibles seulement (table + points viewer), AUCUNE simulation auto
  (`_reset_for_loaded_program`). La simulation se lance au clic **Simuler**.
- Axes externes : la simulation démarre à la **position courante** des axes
  (`_init_ext_axis_state` lit `joint.value` ; le robot, lui, part de HOME via la motion
  HOME_START des réglages de génération).
- Base effective recalculée au moment de Simuler (position des axes au démarrage).

### Calcul segment par segment + incrémental
- `_simulate_motion_list(motions, build_cache=True)` construit le cache incrémental
  PENDANT la passe (frontières exactes ; l'état robot/TCP/axes est connu au début de
  chaque segment). L'ancien `_build_motion_cache` (groupage par numéro de ligne, cassé
  pour les motions dérivées ligne 0) a été supprimé.
- `_MotionSimCacheEntry` valide signature + joints départ + pose départ + **snapshot
  axes externes départ** ; temps relatifs au DÉBUT du mouvement (fix dérive temporelle).
- `simulate_program_incremental` : entrée 0 du cache préservée, cascade automatique si
  l'état de fin change. Édition d'un LIN → seuls ce segment et les suivants impactés
  sont resimulés (validé par test : édition motion 2 → motions 2+3 resimulés, ~0,24 s
  vs 0,7 s full sur 1945 samples).
- Contrôleur : `_dirty_motion_indices` (None = tout) accumulé à l'édition ; au clic
  Simuler → incrémental si édits ciblés, complet sinon.
- Résultat du mode opposé (cartésien ↔ articulaire) calculé **à la demande** au
  changement de mode (`_ensure_motion_mode_results`) via `_derived_simulator` (instance
  dédiée pour ne pas écraser le cache principal ; idem compensation).
- Combo mode table : data « JOINT » corrigée en « ARTICULAR » (le mode articulaire ne
  fonctionnait pas via le combo).

### Dialog « Paramètres » (non bloquant, viewer interactif)
`ProgramSettingsDialog` réécrit : scroll + sections dans l'ordre :
1. **Base programme** (`ProgramBaseSectionWidget`) : repère de référence
   (Monde / Robot / Fichier programme / Pièce / Manuel / Axe externe…) + offsets XYZABC.
   Offset appliqué `T_source @ T_offset`. Spinboxes `keyboardTracking(False)` → commit
   sur Entrée / perte de focus / flèches uniquement. Mise à jour live des cibles.
2. **Outil** (`ProgramToolSectionWidget`) : Courant (config) / Programme / Personnalisé
   (XYZABC vs flange). `_tool_source` ∈ {CURRENT, PROGRAM, CUSTOM} + `_custom_tool_pose`.
3. **Vitesse trajectoire** : checkbox vitesse constante + mm/s (dans
   `ProgramGenerationWidget`, champ `constant_speed_mmps` de `ProgramGenerationSettings`,
   None = vitesses du programme). Appliquée aux LIN/CIRC NORMAL à la simulation et au KRL.
4. **Approximation** : défaut C_DIS = 1.0 (`ProgramGenerationSettings`).
5. **Approche/Retrait** (existant). 6. **Header + preview KRL** (existant).
- L'ancien `ProgramBaseDialog` (bloquant) et le combo outil de la table sont supprimés.
- État persisté étendu (`get/load_base_config_state`) : manual_base, base_ext_axis_id,
  tool_source, custom_tool.

### Table & génération KRL
- Combo « Repère » de la table = Programme / Robot / Monde : **expression** des
  coordonnées seulement (ne déplace pas les cibles) et définit le `$BASE` du KRL.
- `_program_for_generation()` ré-exprime les cibles cartésiennes dans le repère choisi
  (matrices), retourne la pose `$BASE` associée.
- `generate_kuka_src_text(..., tool_pose=, base_pose=)` : `DEF <nom_programme>()`,
  `$APO.CDIS = x` ou `$APO.CVEL = x`, `$VEL.CP`, `$BASE = {FRAME: ...}`,
  `$TOOL = {FRAME: ...}`, mention ` C_DIS`/` C_VEL` après chaque LIN/CIRC.

## Reste à faire / à vérifier en lançant l'app

1. **Test manuel complet** (rien n'a été validé en GUI) :
   - import .src → cibles visibles sans trajectoire ; Simuler → trajectoire ; Play →
     home → axes externes → trajectoire → home → axes à 0 ; vérifier que la trajectoire
     ne bouge plus quand le rail bouge et que le playback ne se bloque plus.
   - ajout d'un mouvement d'axe externe (rail 2000 mm, positionneur 90°) via « Ajouter ».
2. **Repère Monde de la table** : `_motion_target_to_keypoint` calcule la pose monde avec
   l'état courant des axes — après un mouvement d'axes pendant playback, la table n'est
   pas re-rafraîchie (acceptable, à confirmer).
3. **Cibles attachées au positionneur pendant le playback** : les keypoints viewer sont
   figés en monde au moment de la simulation (pas suivis pendant la rotation du
   positionneur). À améliorer si besoin.
4. **MGI vs calibration** : inchangé (bug connu existant).
5. **Compensation** : non testée après refactor (passe par `_derived_simulator`).
6. **`_rebuild_derived_motions`** (HOME/approche/retrait) toujours appliqué seulement aux
   programmes importés FAO — décider s'il faut l'étendre aux .src chargés.
7. Vitesse constante : appliquée aussi aux approches/retraits ? Actuellement non
   (uniquement rôle NORMAL).
8. Nettoyage éventuel : `_estimate_total_path_length_mm` ne tient pas compte des axes
   externes (pas adaptatif sur ces mouvements, sans gravité).

## Prompt de reprise pour la prochaine session

> Lis docs/REFACTOR_SIMULATION_PROGRAMME.md (état du refactoring de la simulation
> programme, commit 3212fbc). Lance l'application (.venv\Scripts\python.exe main.py),
> teste le scénario : import d'un programme .src (cibles sans trajectoire), ajout d'un
> mouvement d'axe externe (rail à 2000 mm + positionneur à 90° puis retour à 0), clic
> Simuler, playback complet. Corrige les bugs rencontrés (en particulier : affichage
> monde des trajectoires/cibles, guard playback, dialog Paramètres non bloquant,
> simulation incrémentale au clic Simuler) et traite la liste « Reste à faire » du doc.
