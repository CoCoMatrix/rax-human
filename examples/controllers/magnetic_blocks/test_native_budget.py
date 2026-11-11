import unittest
import controller
class NativeBudget(unittest.TestCase):
    def test_current_frontend_contract_without_primary_model_budget(self):
        controller._budget({'wall_budget':{'remaining_seconds':500,'finish_reserve_seconds':60,'phase':'active'}},120)
    def test_native_finish_boundary_still_blocks(self):
        with self.assertRaises(RuntimeError):
            controller._budget({'wall_budget':{'remaining_seconds':179,'finish_reserve_seconds':60,'phase':'active'}},120)
    def test_missing_or_malformed_native_budget_blocks(self):
        for budget in [{},{'remaining_seconds':1000},{'remaining_seconds':True,'finish_reserve_seconds':60,'phase':'active'}, {'remaining_seconds':1000,'finish_reserve_seconds':float('nan'),'phase':'active'},{'remaining_seconds':1000,'finish_reserve_seconds':60,'phase':'wind_down'}]:
            with self.assertRaises(RuntimeError):controller._budget({'wall_budget':budget})
if __name__=='__main__':unittest.main()
