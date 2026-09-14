"""Regression contracts for calibration without held-out leakage or covariance inflation."""
import numpy as np
import pytest

from imuFactors.noise_calibration import (
    BASE_ALPHA, LikelihoodBlock, covariance_at, fit_noise, likelihood,
    position_drive_covariance, training_sequences,
)


def test_loso_keeps_all_intervals_of_heldout_sequence_out():
    names=['MH01','MH02','MH03','V101','V102']
    assert training_sequences(names,'MH02','group_loso') == ['MH01','MH03']
    assert training_sequences(names,'V101','group_loso') == ['V102']
    assert training_sequences(names,'MH02','global_loso') == ['MH01','MH03','V101','V102']


def exact_moment_errors(covariance):
    # 18 symmetric errors have zero mean and exactly the desired second moment.
    return np.concatenate([3*np.linalg.cholesky(covariance).T,
                           -3*np.linalg.cholesky(covariance).T])


@pytest.mark.parametrize('alpha,q',[(4.2,0.),(12.,2e-5),(8.4,1e-8)])
def test_profiled_fit_recovers_known_covariance(alpha,q):
    base=np.diag(np.arange(1.,10.))*1e-7
    drive=position_drive_covariance(18,.5)
    covariance=covariance_at(np.repeat(base[None],18,axis=0),drive,alpha,q)
    block=LikelihoodBlock.from_arrays(exact_moment_errors(covariance[0]),
                                      np.repeat(base[None],18,axis=0),drive)
    result=fit_noise([block])
    assert result['alpha'] == pytest.approx(alpha,rel=1e-5)
    assert result['integration_covariance'] == pytest.approx(q,rel=1e-4,abs=1e-12)
    assert likelihood([block],alpha*100,q) > result['training_nll']


def test_whitened_likelihood_matches_direct_gaussian_and_equal_block_weights():
    rng=np.random.default_rng(8)
    blocks=[];direct=[]
    for count in (3,9):
        errors=rng.normal(size=(count,9))
        matrix=rng.normal(size=(count,9,9))
        base=matrix @ matrix.transpose(0,2,1) + np.eye(9)
        drive=position_drive_covariance(count,.5)
        covariance=covariance_at(base,drive,12.,2.)
        direct.append(.5*np.mean(np.linalg.slogdet(covariance)[1]+
                       np.einsum('ni,ni->n',errors,np.linalg.solve(covariance,errors[...,None])[...,0])+9*np.log(2*np.pi)))
        blocks.append(LikelihoodBlock.from_arrays(errors,base,drive))
    assert likelihood(blocks,12.,2.) == pytest.approx(np.mean(direct))
    repeat=LikelihoodBlock(np.repeat(blocks[0].eigenvalues,5,axis=0),
                           np.repeat(blocks[0].squared_errors,5,axis=0),
                           np.repeat(blocks[0].logdet_base,5))
    assert likelihood([repeat,blocks[1]],12.,2.) == pytest.approx(np.mean(direct))


@pytest.mark.parametrize('alpha,q',[(0.,0.),(-1.,0.),(1.,-1.),(np.nan,1.),(1.,np.inf)])
def test_invalid_noise(alpha,q):
    with pytest.raises(ValueError):
        covariance_at(np.eye(9),np.eye(9),alpha,q)
