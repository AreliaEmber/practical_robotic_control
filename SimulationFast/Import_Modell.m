%% Onshape → Simscape Multibody Import Script

% URL der Onshape‑Assembly
assemblyURL = "https://cad.onshape.com/documents/1f3dd13ae488f43b18d0499d/w/75ceefff6c2ced1d9b1151bd/e/efc428b02cfb9b5e42b55e5a";

% Export der Onshape‑CAD‑Assembly
xmlFile = smexportonshape(assemblyURL);

% Import in Simscape Multibody
smimport(xmlFile);

% New Name
newModelName = 'Tumbller_1';
save_system(gcs, newModelName);