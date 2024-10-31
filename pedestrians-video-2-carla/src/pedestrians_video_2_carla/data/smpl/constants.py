import os

SMPL_BODY_MODEL_DIR = os.path.realpath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', '..',
    'models', 'smpl-x', 'smplx_locked_head'))
SMPL_MODELS = {
    'male': os.path.join('male', 'model.npz'),
    'female': os.path.join('female', 'model.npz'),
    'neutral': os.path.join('neutral', 'model.npz')
}
AMASS_DIR = 'AMASS'
