 Contexte

 L'onglet Programme permet aujourd'hui de charger/éditer un programme robot KUKA (.src),
 de simuler la trajectoire et d'exporter une version compensée. On veut en faire un véritable
 post-processeur : importer du code FAO (APTSource et CATNCcode) pour en extraire les
 cibles et les vitesses, les transformer en programme robot (rep. pièce + outil courant par
 défaut), encadrer le tout par des mouvements HOME et des séquences d'approche/retrait,
 gérer le positionnement d'axes externes, ajouter un paramètre d'approximation c_dis/c_vel
 purement pour la génération, fournir un header KRL personnalisable et une prévisualisation
 KRL affichables à la demande, et enfin ne recalculer que les segments impactés lors d'une
 édition de cible.

 Faits d'architecture établis (à respecter)

 - L'onglet Programme utilise son propre modèle et son propre simulateur, distincts du pipeline
 trajectory_engine/ (qui sert l'onglet Trajectoire). Le modèle est models/robot_program.py
 (RobotProgram → list[RobotProgramMotion]), le simulateur est utils/program_simulator.py
 (simulate_program → _simulate_motion_list → _simulate_motion par motion, temps cumulatif),
 l'I/O KRL est utils/robot_program_kuka.py, le contrôleur est controllers/program_controller.py.
 - L'export KRL actuel patche le source_text existant (export_kuka_src_program) : il réécrit
 les lignes de mouvement et la ligne $BASE, sans header généré. Cela convient pour un .src
 chargé. Pour un programme généré depuis APT/CATNC il faut un nouveau générateur KRL
 « from scratch » (header + DEF/END + HOME + approche + cibles + retrait + axes externes).
 - RobotProgramMotion (frozen dataclass) porte déjà : mode, target, line_number, source,
 base_pose, tool_pose, via_target, cp_speed_mps. RobotProgramTarget porte target_type
 (CARTESIAN/JOINT), cartesian_pose: Pose6, joint_angles: JointAngles6.
 - Le contrôleur sait déjà : base MANUAL/WORKPIECE + offset (_compute_effective_base), outil
 CURRENT/PROGRAM, édition de la base (ProgramBaseDialog) et de l'orientation outil globale
 (ProgramToolOrientationDialog), et le recompute complet via _recompute_current_program.
 - Axes externes : models/external_axes_model.py (axes positionnés ExternalAxisMountMode.POSITIONED,
 joints LINEAR/ROTARY, get_axes(), get_axis_joint_value(), compute_world_transforms()).
 Aucune émission KRL E1..E6 n'existe encore.
 - Position HOME : robot_model.get_home_position() → JointAngles6.

 Décisions produit (validées avec l'utilisateur)

 1. Orientation à l'import : importer le vecteur axe-outil i,j,k quand la source le fournit
 (compose une orientation A/B/C dont l'axe Z outil = vecteur normalisé). Sinon orientation par
 défaut éditable. L'éditeur d'orientation globale existant reste disponible pour écraser.
 2. Approche/retrait : axe de décalage choisi par séquence — Z-outil ou un axe fixe
 pièce X/Y/Z — avec distance (mm) et vitesse.
 3. Axes externes : mouvement dédié de positionnement (PTP des E-axes seuls, positionné/async),
 inséré comme ligne propre dans la table, distinct des cibles cartésiennes.
 4. Header KRL : modèle par défaut versionné dans default_data/, override dans user_data/,
 reset possible.

 ---
 Conventions à respecter (rappel CLAUDE.md / CODING_GUIDELINES)

 - Pas de dict[str, Any] ni de list[float]/tuple pour une donnée métier de taille fixe →
 types dédiés dans models/types/. Réutiliser Pose6, XYZ3, JointAngles6.
 - Angles en degrés côté UI/JSON/KRL, radians dans les calculs internes. Longueurs en mm.
 - Pattern Qt strict Modèle ↔ Contrôleur ↔ Vue via pyqtSignal : les widgets émettent, le contrôleur mute.
 - from __future__ import annotations. Conversions brutes uniquement aux frontières (KRL, JSON, Qt).
 - Le MGD/MGI nominal doit donner exactement les mêmes résultats qu'avant. Vérifier après coup.
 - venv obligatoire : .venv\Scripts\python.exe. Commits français, pas de push.
 - Mémoire orientation KUKA : composer une matrice de rotation, ne jamais écraser l'Euler quand
 on dérive A/B/C d'un vecteur (voir [[orientation-jog-kuka-behaviour]]).

 ---
 1. Extensions du modèle de données (models/)

 1.1 models/robot_program.py

 - ProgramOrigin(Enum) : LOADED_KRL, IMPORTED_APT, IMPORTED_CATNC, BUILT.
 Ajouter origin: ProgramOrigin = LOADED_KRL à RobotProgram. Pilote : génération « from
 scratch » + header si origin != LOADED_KRL, sinon patch + header ignoré.
 - MotionRole(Enum) : NORMAL, HOME_START, HOME_END, APPROACH, RETRACT,
 EXTERNAL_SETUP. Ajouter role: MotionRole = NORMAL à RobotProgramMotion. Les rôles non
 NORMAL sont générés/dérivés et affichés en lignes verrouillées (non supprimables) dans la table.
 - Approximation : nouveau RobotProgramMotionMode inchangé ; ajouter un type dédié
 models/types/motion_approximation.py → MotionApproximation (frozen) :
 mode: ApproximationMode (NONE/C_DIS/C_VEL), value: float (mm pour C_DIS, % pour C_VEL).
 Ajouter approximation: MotionApproximation = MotionApproximation.none() à RobotProgramMotion.
 Génération KRL uniquement — ignoré par le simulateur et la trajectoire.
 - Axes externes (mouvement dédié) : ajouter mode RobotProgramMotionMode.EXTERNAL_AXIS.
 Nouveau type models/types/external_axis_program_target.py :
   - ExternalAxisJointValue (frozen) : axis_id: str, joint_index: int,
 value: float (mm ou deg selon le type de joint), joint_type: ExternalAxisJointType.
   - ExternalAxisProgramTarget (frozen) : values: list[ExternalAxisJointValue] (taille variable
 → liste acceptable ici car le nombre d'axes/joints n'est pas fixe).
 Ajouter external_axis_target: ExternalAxisProgramTarget | None = None à RobotProgramMotion
 (renseigné pour les motions EXTERNAL_AXIS ; pour les autres motions, conserver la dernière
 position d'axe externe connue comme état courant lors de la simulation/affichage).

 1.2 Séquences approche/retrait et header — types dédiés

 - models/types/approach_retract.py :
   - ApproachAxisRef(Enum) : TOOL_Z, PIECE_X, PIECE_Y, PIECE_Z.
   - ApproachRetractConfig (frozen) : enabled: bool, axis_ref: ApproachAxisRef,
 distance_mm: float, speed_mps: float. Deux instances (approche / retrait) stockées au niveau
 programme (cf. 1.3).
 - models/program_generation_settings.py (@dataclass, non frozen, état édité par l'utilisateur,
 porté par le contrôleur / sérialisé) :
 home_enabled: bool (vrai par défaut, imposé), approach: ApproachRetractConfig,
 retract: ApproachRetractConfig, header_text: str, default_approximation: MotionApproximation.

 1.3 Persistance

 - Header : default_data/programs/krl_header_template.src (modèle versionné) + override
 user_data/programs/krl_header_template.src. Loader dédié models/krl_header_file.py
 (load_header_template() → lit l'override sinon le défaut ; save_header_template(text) →
 écrit l'override ; reset_header_template() → supprime l'override). Ajouter la création du
 dossier/raccourci dans utils/user_data_paths.py si nécessaire.
 - Réglages de génération (approche/retrait, home, approximation par défaut, chemin header) :
 étendre la section programme de la session (models/app_session_file.py + MainController),
 debounce 250 ms existant. Conversions brutes concentrées dans le *_file.py.

 ---
 2. Parsers FAO → RobotProgram (utils/)

 Suivre le patron de utils/robot_program_kuka.py (regex, frontière unique, warnings).

 2.1 utils/aptsource_parser.py → load_aptsource_program(path) -> RobotProgram

 - Parser les commandes APT/CLDATA pertinentes : GOTO x,y,z[,i,j,k], FROM, RAPID/RAPIDTO,
 FEDRAT f (mm/min → m/s : /60000), GOHOME, SPINDL (ignoré), CUTCOM/MULTAX (ignorés,
 loggés en warning si non géré).
 - Chaque GOTO → RobotProgramMotion(mode=LINEAR, target=CARTESIAN cartesian_pose),
 RAPID → PTP. Vitesse = dernier FEDRAT actif → cp_speed_mps.
 - Orientation : si i,j,k présents → orientation via _orientation_from_tool_axis()
 (cf. 2.3) ; sinon Pose6 avec A/B/C par défaut (réglable ensuite).
 - origin=ProgramOrigin.IMPORTED_APT, brand=KUKA. line_number = ligne source.

 2.2 utils/catnc_parser.py → load_catnc_program(path) -> RobotProgram

 - CATNCcode = ISO/G-code CATIA. Parser : G00 (rapide→PTP), G01 (linéaire→LIN),
 G02/G03 (arc→CIRCULAR avec point intermédiaire calculé depuis I/J/K), modales X Y Z
 (position courante mise à jour incrémentalement), F (feed mm/min → m/s), S/M/commentaires
 ( ... ) ignorés. Gérer les modes modaux (la position non répétée reste celle courante).
 - Même mapping vers RobotProgramMotion, origin=ProgramOrigin.IMPORTED_CATNC.

 2.3 utils/math_utils.py — helper d'orientation depuis vecteur axe-outil

 - _orientation_from_tool_axis(tool_axis: XYZ3, reference: XYZ3) -> Pose6 (A/B/C deg) :
 composer une matrice de rotation dont l'axe Z = tool_axis normalisé, l'axe X résolu en
 projetant reference (par défaut X pièce) dans le plan ⟂ Z (Gram-Schmidt), Y = Z × X ; puis
 matrice → A/B/C via la conversion ZYX existante (pose_*_to_matrix / inverse de math_utils).
 Ne jamais composer les angles d'Euler à la main (cf. mémoire KUKA). Le degré de liberté en
 rotation autour de Z (indéterminé par i,j,k) est fixé par cette convention de référence,
 documentée dans une docstring.

 ▎ Note migration C++ : ces parsers sont du parsing pur + remplissage de structs → directement
 ▎ transposables. Garder zéro dépendance Qt dans utils/*_parser.py.

 ---
 3. Génération KRL « from scratch » + header (utils/robot_program_kuka.py)

 Ajouter, sans toucher au comportement de export_kuka_src_program (patch) utilisé pour les
 .src chargés :

 - generate_kuka_src_text(program, header_text, settings, external_axes_order) -> str :
 produit un .src complet quand program.origin != LOADED_KRL :
   a. Header (texte du template, tokens substitués : {PROGRAM_NAME}, {DATE}, {BASE},
 {TOOL}, {VEL_CP}, {N_MOTIONS}, {FOLD}…). DEF <name>() … corps … END.
   b. $BASE, $TOOL, $VEL.CP.
   c. PTP HOME (rôle HOME_START).
   d. Éventuel mouvement de positionnement axes externes initial (EXTERNAL_SETUP).
   e. Approche (APPROACH) si activée.
   f. Cibles NORMAL (PTP/LIN/CIRC), avec approximation émise : C_DIS/C_VEL ajouté à la
 ligne de mouvement (ex. LIN {…} C_DIS ou paramétrage $APO), selon syntaxe KUKA standard.
   g. Retrait (RETRACT) si activé.
   h. PTP HOME final (HOME_END).
 - Émission axes externes E1..E6 : _format_external_axis_block(target, external_axes_order) →
 les valeurs ExternalAxisProgramTarget mappées sur E1..E6 dans l'ordre des axes du
 external_axes_model (convention documentée). Mouvement positionné asynchrone : générer un
 bloc PTP {E1 …, E2 …} (ou ASYPTP/$EX_AX_ASYNC pour async — choisir ASYPTP + commentaire
 expliquant « axe positionné non synchronisé »). Étendre _parse_target_block pour lire aussi
 E1..E6 (round-trip).
 - generate_program_to_path(path, program, …) : choisit generate_kuka_src_text si origine
 générée, sinon export_kuka_src_program (patch). Le contrôleur appelle ce point unique.

 ---
 4. HOME + approche/retrait : dérivation des motions (controllers/program_controller.py)

 - Les motions de rôle HOME_START/HOME_END/APPROACH/RETRACT/EXTERNAL_SETUP sont dérivées depuis
 ProgramGenerationSettings + les cibles NORMAL de bord. Centraliser dans
 _rebuild_derived_motions(program, settings) -> RobotProgram :
   - HOME : PTP JOINT = robot_model.get_home_position().
   - APPROACH : pose = première cible cartésienne NORMAL décalée de distance_mm le long de
 axis_ref (Z-outil via la pose outil courante, ou axe pièce X/Y/Z via la base effective).
 Composer la matrice de décalage, ne pas bricoler les Euler.
   - RETRACT : symétrique sur la dernière cible cartésienne NORMAL.
   - Vitesses approche/retrait : cp_speed_mps = config.speed_mps.
 - Ces motions sont insérées dans la liste affichée comme lignes verrouillées (cf. UI §7) et
 régénérées à chaque changement de bord/settings — mais via le recompute incrémental (§6),
 pas un rebuild complet.
 - À l'import APT/CATNC : forcer base_source=WORKPIECE, tool_source=CURRENT (déjà supportés),
 settings.home_enabled=True.

 ---
 5. Axes externes positionnés — saisie (widgets/program_view/)

 - Étendre ProgramTargetDialog : ajouter un mode « Positionnement axe externe » (visible
 seulement si external_axes_model.get_axes() non vide). Le dialog présente un spinbox par
 joint d'axe externe (label + unité « mm »/« ° » depuis ExternalAxisJoint.unit), pré-rempli
 avec la position courante (get_axis_joint_value). Retourne un ExternalAxisProgramTarget.
 - Bouton « Appliquer position courante » réutilisé pour récupérer l'état courant des axes.
 - Le contrôleur (_motion_from_dialog) crée un RobotProgramMotion(mode=EXTERNAL_AXIS, role=NORMAL,
 external_axis_target=…). Simulation (§ simulateur) : un mouvement EXTERNAL_AXIS
 ne déplace pas le TCP robot — il met seulement à jour l'état d'axe externe courant (et donc
 la base/pièce effective si le robot/la pièce est monté dessus, via
 utils/external_axes_kinematics.world_robot_base()), durée dérivée d'une vitesse d'axe par défaut.

 ---
 6. Recompute incrémental (utils/program_simulator.py + contrôleur)

 Objectif : sur add/edit/delete d'une cible, ne recalculer que le motion modifié et, en
 cascade, le suivant si son état de départ change. Conserver la règle « un seul writer ».

 6.1 Cache par motion dans le simulateur

 - Ajouter simulate_program_incremental(program, previous_result, dirty_indices) -> ProgramSimulationResult.
 - Cache interne list[_MotionSimCacheEntry] parallèle aux motions :
 signature (hash des champs métier du motion : mode/target/speed/tool/base — hors
 approximation, qui n'influe pas), start_state (joints + pose de départ), samples_relatifs
 (temps relatifs depuis 0), end_state.
 - Algorithme :
   a. Marquer dirty = dirty_indices (et leurs voisins immédiats pour insert/delete).
   b. Itérer les motions ; pour chaque motion, si signature et start_state inchangés → réutiliser
 l'entrée cache (samples relatifs). Sinon re-simuler ce motion (_simulate_motion) ; si son
 end_state diffère de l'ancien → marquer i+1 dirty (cascade).
   c. Restitcher les temps cumulés : time_s = offset + t_relatif, l'offset s'accumule sur les
 durées. (Un simple changement de vitesse ne touche que les offsets en aval, pas la géométrie.)
 - La compensation (variantes mesurée/compensée) suit la même logique ou reste recalculée à la
 demande via le bouton dédié (ne pas la recalculer à chaque édition).

 6.2 Câblage contrôleur

 - Le contrôleur connaît l'index muté (add/edit/delete passent déjà par _ProgramTargetRef /
 _program_with_inserted|deleted|replaced_motion). Remplacer l'appel à
 _recompute_current_program() (full) par un _recompute_incremental(dirty_indices) qui appelle
 simulate_program_incremental. Garder un chemin full pour : chargement, changement de base,
 changement d'orientation globale, changement de réglages génération, bouton « Simuler ».
 - Régénérer ensuite uniquement les segments d'affichage 3D impactés (le cache segments
 _nominal_segments_cache / _nom_seg_pts peut être mis à jour par plage, mais en première
 itération on peut reconstruire l'affichage à partir des samples — l'important métier est le
 non-recalcul de la simulation).

 ---
 7. UI : table, header, preview KRL (widgets/program_view/, views/program_view.py)

 - Table ProgramKeypointsWidget :
   - Lignes verrouillées (HOME/APPROACH/RETRACT/EXTERNAL_SETUP) : non éditables/supprimables,
 style grisé, libellé de rôle dans la colonne « Cible ».
   - Nouvelle colonne « Approx » (NONE / C_DIS=… / C_VEL=…), éditée via le dialog.
   - Lignes EXTERNAL_AXIS : afficher « Axe ext. » + valeurs E synthétiques.
 - Dialog d'ajout : sélecteur de type {Cartésien, Joint, Positionnement axe externe},
 champs approximation (mode + valeur), réutilise le default_approximation des settings.
 - Zone header (sur demande) : bouton bascule « Afficher/Masquer header » dans
 ProgramActionsWidget (ou le header widget) → affiche un QPlainTextEdit (caché par défaut)
 contenant header_text. Boutons « Enregistrer header » (→ save_header_template) et
 « Réinitialiser » (→ reset_header_template). Émet headerChanged ; le contrôleur met à jour
 settings.header_text. Masqué par défaut pour ne pas prendre de place.
 - Preview KRL (sur demande) : bouton bascule « Afficher/Masquer preview KRL » → QPlainTextEdit
 lecture seule rempli par generate_kuka_src_text(current_program, header_text, settings, …).
 Rafraîchi à chaque recompute (debounce). Permet de vérifier avant Enregistrer.
 - Réglages approche/retrait/home : petit panneau (combo axis_ref, distance, vitesse, cases
 enabled, case home cochée+désactivée car imposée) dans ProgramActionsWidget ou un
 program_generation_widget.py dédié. Émet generationSettingsChanged → contrôleur met à jour
 les settings + _rebuild_derived_motions + recompute (full, c'est une bordure).
 - Import : étendre le filtre du dialog de chargement (header widget) :
 KUKA (*.src);;APT Source (*.apt *.aptsource *.cls *.cl);;CATNC (*.nc *.cnc *.mpf *.gcode);;Tous (*.*)
 et router selon extension vers load_kuka_src_program / load_aptsource_program /
 load_catnc_program dans program_controller._load_program_from_path.

 ---
 8. Fichiers à créer / modifier (récap)

 Créer
 - utils/aptsource_parser.py, utils/catnc_parser.py
 - models/types/motion_approximation.py, models/types/external_axis_program_target.py,
 models/types/approach_retract.py
 - models/program_generation_settings.py, models/krl_header_file.py
 - default_data/programs/krl_header_template.src (modèle KRL par défaut)
 - (option) widgets/program_view/program_generation_widget.py

 Modifier
 - models/robot_program.py (ProgramOrigin, MotionRole, champs approximation/external sur motion)
 - utils/robot_program_kuka.py (générateur from-scratch + E1..E6 + approximation)
 - utils/program_simulator.py (cache + simulate_program_incremental, motion EXTERNAL_AXIS)
 - utils/math_utils.py (_orientation_from_tool_axis)
 - controllers/program_controller.py (import routing, settings, motions dérivées, recompute incrémental)
 - widgets/program_view/program_keypoints_widget.py, program_target_dialog.py,
 program_actions_widget.py, views/program_view.py
 - models/app_session_file.py + MainController (persistance settings), utils/user_data_paths.py

 ---
 9. Vérification (end-to-end, venv obligatoire)

 Lancer : .venv\Scripts\python.exe main.py

 1. Non-régression MGD/MGI : charger un .src existant, comparer les samples nominaux
 (joints/poses) à l'état actuel — doivent être identiques. Idéalement via un petit script de
 comparaison utilisant simulate_program sur tools/_regression_snapshot.json si exploitable.
 2. Import APT : charger un .apt de test → cibles LIN + vitesses correctes, base=pièce,
 outil=courant, orientation dérivée de i,j,k cohérente (TCP pointe le long du vecteur).
 3. Import CATNC : charger un .nc G00/G01/G02 → PTP/LIN/CIRC corrects, feed converti en m/s.
 4. HOME + approche/retrait : vérifier lignes verrouillées HOME début/fin ; changer axe
 (Z-outil vs pièce X) et distance → la pose d'approche/retrait se décale correctement en 3D.
 5. Axe externe : avec une config d'axes externes chargée, ajouter un mouvement de
 positionnement → valeurs E saisies, simulation ne bouge pas le TCP, KRL émet E1..E6.
 6. Approximation : régler C_DIS sur une cible → présent dans la preview KRL, aucun effet
 sur la simulation/3D (samples identiques avec/sans approximation).
 7. Header : afficher la zone, modifier, enregistrer, recharger l'app → header persistant ;
 « Réinitialiser » restaure le défaut ; header ignoré pour un .src chargé directement.
 - utils/aptsource_parser.py, utils/catnc_parser.py
 - models/types/motion_approximation.py, models/types/external_axis_program_target.py,
 models/types/approach_retract.py
 - models/program_generation_settings.py, models/krl_header_file.py
 - default_data/programs/krl_header_template.src (modèle KRL par défaut)
 - (option) widgets/program_view/program_generation_widget.py

 Modifier
 - models/robot_program.py (ProgramOrigin, MotionRole, champs approximation/external sur motion)
 - utils/robot_program_kuka.py (générateur from-scratch + E1..E6 + approximation)
 - utils/program_simulator.py (cache + simulate_program_incremental, motion EXTERNAL_AXIS)
 - utils/math_utils.py (_orientation_from_tool_axis)
 - controllers/program_controller.py (import routing, settings, motions dérivées, recompute incrémental)
 - widgets/program_view/program_keypoints_widget.py, program_target_dialog.py,
 program_actions_widget.py, views/program_view.py
 - models/app_session_file.py + MainController (persistance settings), utils/user_data_paths.py

 ---
 9. Vérification (end-to-end, venv obligatoire)

 Lancer : .venv\Scripts\python.exe main.py

 1. Non-régression MGD/MGI : charger un .src existant, comparer les samples nominaux
 (joints/poses) à l'état actuel — doivent être identiques. Idéalement via un petit script de
 comparaison utilisant simulate_program sur tools/_regression_snapshot.json si exploitable.
 2. Import APT : charger un .apt de test → cibles LIN + vitesses correctes, base=pièce,
 outil=courant, orientation dérivée de i,j,k cohérente (TCP pointe le long du vecteur).
 3. Import CATNC : charger un .nc G00/G01/G02 → PTP/LIN/CIRC corrects, feed converti en m/s.
 4. HOME + approche/retrait : vérifier lignes verrouillées HOME début/fin ; changer axe
 (Z-outil vs pièce X) et distance → la pose d'approche/retrait se décale correctement en 3D.
 5. Axe externe : avec une config d'axes externes chargée, ajouter un mouvement de
 positionnement → valeurs E saisies, simulation ne bouge pas le TCP, KRL émet E1..E6.
 6. Approximation : régler C_DIS sur une cible → présent dans la preview KRL, aucun effet
 sur la simulation/3D (samples identiques avec/sans approximation).
 7. Header : afficher la zone, modifier, enregistrer, recharger l'app → header persistant ;
 « Réinitialiser » restaure le défaut ; header ignoré pour un .src chargé directement.
 8. Preview KRL : afficher/masquer, vérifier que Enregistrer produit le même texte que la preview.
 9. Recompute incrémental : éditer la vitesse d'une cible au milieu → vérifier (log/temps de
 calcul, ou compteur de motions re-simulés) que seuls le motion édité (et le suivant si l'état de
 fin change) sont recalculés, pas toute la trajectoire.

 ---
 10. Ordre de livraison suggéré (commits français séparés, sans push)

 1. Types & modèle (robot_program, types/* nouveaux, settings, header_file).
 2. Parsers APT + CATNC + helper orientation, + routing import dans le contrôleur.
 3. Générateur KRL from-scratch + header + E1..E6 + approximation (I/O).
 4. Motions dérivées HOME/approche/retrait + réglages + UI panneau.
 5. Axes externes : dialog + simulation EXTERNAL_AXIS + émission KRL.
 6. Recompute incrémental (cache simulateur + câblage contrôleur).
 7. UI finitions : colonne Approx, lignes verrouillées, zone header & preview KRL togglables.

 Après chaque étape : relancer le test de non-régression MGD nominal (point 9.1).