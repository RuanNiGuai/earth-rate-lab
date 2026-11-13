"""Exact installed accelerometer/g-sens coordinates; no device I/O.

Old snapshots can be used for explicitly labelled diagnostics only. They cannot
establish an installed-firmware baseline or authorize a parameter selection.
"""
import numpy as np


def transform(snapshot, *, allow_unknown=False):
    a = snapshot.get('accelerometer', {})
    if not a.get('known'):
        if allow_unknown:
            return np.zeros(3), np.eye(3)
        raise ValueError('Installed acceleration transform unknown; read a new snapshot')
    b, matrix = np.asarray(a['offset_g']), np.asarray(a['matrix'])
    if b.shape != (3,) or matrix.shape != (3, 3) or not np.isfinite(b).all() or not np.isfinite(matrix).all() or np.linalg.cond(matrix) > 1e6:
        raise ValueError('Invalid installed acceleration transform')
    return b, matrix


def installed_accel(raw, snapshot, *, allow_unknown=False):
    b, a = transform(snapshot, allow_unknown=allow_unknown)
    return (np.asarray(raw) - b) @ a.T


def raw_gsens(snapshot, *, allow_unknown=False):
    """Effective raw-coordinate linear term; callers must also account for B.

    In difference/turn integrals the constant cancels. For absolute rates use
    installed_accel() directly, including its offset.
    """
    _, a = transform(snapshot, allow_unknown=allow_unknown)
    return np.asarray(snapshot['production']['gsens_dps_per_g']).reshape(3, 3) @ a


def convert_raw_model(raw_m, raw_b, snapshot):
    """Convert g = B_raw + M_raw*a_raw to B_cal + M_cal*a_cal.

    Returns both terms. Installing just M_cal would omit M_raw*b.
    This is analysis only: CALWIRE has no atomic M/B/accel transaction.
    """
    b, a = transform(snapshot)
    m = np.asarray(raw_m).reshape(3, 3)
    return m @ np.linalg.inv(a), np.asarray(raw_b) + m @ b
