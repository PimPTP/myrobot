from std_srvs.srv import Trigger

class ServoSrv:
    def __init__(self, node, tuna):

        self.node = node
        self.tuna = tuna

        self.node.declare_parameter('rw_id', 0)
        self.node.declare_parameter('rw_addr', 0)
        self.node.declare_parameter('rw_value', 0)
 
        self.node.create_service(Trigger, 'listservos', self.list_servos)
        self.node.create_service(Trigger, 'listregs', self.list_regs)
        self.node.create_service(Trigger, 'readreg', self.read_reg)
        self.node.create_service(Trigger, 'writereg', self.write_reg)

    def list_servos(self, req, res):
        servos = self.tuna.listServos()
        if servos:
            res.success = True
            res.message = ','.join(f"{s['id']}" for s in servos)
        else:
            res.success = False
            res.message = 'No servos found'
        return res

    def list_regs(self, req, res):
        sid = self.node.get_parameter('rw_id').value
        if sid == 0:
            res.success = False
            res.message = '00'
            return res
        regs = self.tuna.listRegs(sid)
        if not regs:
            res.success = False
            res.message = f'Servo {sid} not responding'
            return res
        res.success = True
        res.message = '; '.join(f"{r['addr']}:{r['name']}={r['value']}" for r in regs)
        return res

    def read_reg(self, req, res):
        sid = self.node.get_parameter('rw_id').value
        addr = self.node.get_parameter('rw_addr').value
        value = self.tuna.readReg(sid, addr)
        res.success = value is not None
        res.message = str(value if value is not None else '00')
        return res

    def write_reg(self, req, res):
        sid = self.node.get_parameter('rw_id').value
        addr = self.node.get_parameter('rw_addr').value
        value  = self.node.get_parameter('rw_value').value
        ans = self.tuna.writeReg(sid, addr, value)
        res.success = bool(ans)
        res.message = str(value if value is not None else '00')
        return res