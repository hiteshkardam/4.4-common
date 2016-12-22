/*
 * f_serial.c - generic USB serial function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>

#include "u_serial.h"


/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

#define GSERIAL_IOCTL_MAGIC		'G'
#define GSERIAL_SET_XPORT_TYPE		_IOW(GSERIAL_IOCTL_MAGIC, 0, u32)
#define GSERIAL_SMD_WRITE		_IOW(GSERIAL_IOCTL_MAGIC, 1, \
					struct ioctl_smd_write_arg_type)

#define GSERIAL_SET_XPORT_TYPE_TTY 0
#define GSERIAL_SET_XPORT_TYPE_SMD 1

#define GSERIAL_BUF_LEN  256
#define GSERIAL_NO_PORTS 8 

struct ioctl_smd_write_arg_type {
	char		*buf;
	unsigned int	size;
};

struct f_gser {
	struct gserial			port;
	u8				data_id;
	u8				port_num;

	u8				online;
	enum transport_type		transport;

	atomic_t			ioctl_excl;
	atomic_t			open_excl;

	u8				pending;
	spinlock_t			lock;
	struct usb_ep			*notify;
	struct usb_request		*notify_req;

	struct usb_cdc_line_coding	port_line_coding;

	/* SetControlLineState request */
	u16				port_handshake_bits;
#define ACM_CTRL_RTS	(1 << 1)	/* unused with full duplex */
#define ACM_CTRL_DTR	(1 << 0)	/* host is ready for data r/w */

	/* SerialState notification */
	u16				serial_state;
#define ACM_CTRL_OVERRUN	(1 << 6)
#define ACM_CTRL_PARITY		(1 << 5)
#define ACM_CTRL_FRAMING	(1 << 4)
#define ACM_CTRL_RI		(1 << 3)
#define ACM_CTRL_BRK		(1 << 2)
#define ACM_CTRL_DSR		(1 << 1)
#define ACM_CTRL_DCD		(1 << 0)
};

static unsigned int no_char_bridge_ports;
static unsigned int no_tty_ports;
static unsigned int no_smd_ports;
static unsigned int no_hsic_sports;
static unsigned int nr_ports;
static unsigned int gser_next_free_port;

enum fserial_func_type {
	USB_FSER_FUNC_NONE,
	USB_FSER_FUNC_SERIAL,
	USB_FSER_FUNC_MODEM,
	USB_FSER_FUNC_MODEM_MDM,
	USB_FSER_FUNC_ACM,
	USB_FSER_FUNC_AUTOBOT,
};

static struct port_info {
	enum transport_type	transport;
	enum fserial_func_type serial_type; 
	unsigned		port_num;
	unsigned char		client_port_num;
	struct f_gser		*gser_ptr;
	bool			dun_w_softap_enable;
} gserial_ports[GSERIAL_NO_PORTS];

static int gser_open_dev(struct inode *ip, struct file *fp);
static int gser_release_dev(struct inode *ip, struct file *fp);
static long gser_ioctl(struct file *fp, unsigned cmd, unsigned long arg);
static void gser_ioctl_set_transport(struct f_gser *gser,
				unsigned int transport);


static const struct file_operations gser_fops = {
	.owner = THIS_MODULE,
	.open = gser_open_dev,
	.release = gser_release_dev,
	.unlocked_ioctl = gser_ioctl,
};

static struct miscdevice gser_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_serial_device",
	.fops = &gser_fops,
};

static inline struct f_gser *func_to_gser(struct usb_function *f)
{
	return container_of(f, struct f_gser, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0x51,
	.bInterfaceProtocol =	1,
	
};

/* full speed support: */

static struct usb_endpoint_descriptor gser_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_fs_function[] = {
	(struct usb_descriptor_header *) &gser_interface_desc,
	(struct usb_descriptor_header *) &gser_fs_in_desc,
	(struct usb_descriptor_header *) &gser_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor gser_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_hs_function[] = {
	(struct usb_descriptor_header *) &gser_interface_desc,
	(struct usb_descriptor_header *) &gser_hs_in_desc,
	(struct usb_descriptor_header *) &gser_hs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor gser_ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor gser_ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor gser_ss_bulk_comp_desc = {
	.bLength =              sizeof gser_ss_bulk_comp_desc,
	.bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *gser_ss_function[] = {
	(struct usb_descriptor_header *) &gser_interface_desc,
	(struct usb_descriptor_header *) &gser_ss_in_desc,
	(struct usb_descriptor_header *) &gser_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &gser_ss_out_desc,
	(struct usb_descriptor_header *) &gser_ss_bulk_comp_desc,
	NULL,
};

static struct usb_string modem_string_defs[] = {
	[0].s = "HTC Modem",
	[1].s = "HTC 9k Modem",
	{  } 
};

static struct usb_gadget_strings modem_string_table = {
	.language =		0x0409,	
	.strings =		modem_string_defs,
};

static struct usb_gadget_strings *modem_strings[] = {
	&modem_string_table,
	NULL,
};

static struct usb_string gser_string_defs[] = {
	[0].s = "HTC Serial",
	{  } 
};

static struct usb_gadget_strings gser_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		gser_string_defs,
};

static struct usb_gadget_strings *gser_strings[] = {
	&gser_string_table,
	NULL,
};

int gport_setup(struct usb_configuration *c)
{
	int ret = 0;
	int port_idx;
	int i;

	pr_debug("%s: no_tty_ports: %u no_smd_ports: %u no_hsic_sports: %u nr_ports: %u\n",
		__func__, no_tty_ports, no_smd_ports, no_hsic_sports, nr_ports);

	if (no_tty_ports) {
		for (i = 0; i < nr_ports; i++) {
			if (gserial_ports[i].transport == USB_GADGET_XPORT_TTY)
				ret = gserial_alloc_line(
						&gserial_ports[i].client_port_num);
			if (ret)
				return ret;
		}
	}

	if (no_char_bridge_ports)
		gbridge_setup(NULL, no_char_bridge_ports); 
	if (no_smd_ports)
		ret = gsmd_setup(NULL, no_smd_ports); 
	if (no_hsic_sports) {
		port_idx = ghsic_data_setup(no_hsic_sports, USB_GADGET_SERIAL);
		if (port_idx < 0)
			return port_idx;

		for (i = 0; i < nr_ports; i++) {
			if (gserial_ports[i].transport ==
					USB_GADGET_XPORT_HSIC) {
				gserial_ports[i].client_port_num = port_idx;
				port_idx++;
			}
		}

		/*clinet port num is same for data setup and ctrl setup*/
		ret = ghsic_ctrl_setup(no_hsic_sports, USB_GADGET_SERIAL);
		if (ret < 0)
			return ret;
		return 0;
	}

	return ret;
}

void gport_cleanup(void)
{
	int i;

	for (i = 0; i < no_tty_ports; i++)
		gserial_free_line(gserial_ports[i].client_port_num);
}

static int gport_connect(struct f_gser *gser)
{
	unsigned	port_num;
	int		ret;

	pr_debug("%s: transport: %s f_gser: %pK gserial: %pK port_num: %d\n",
			__func__, xport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	port_num = gserial_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_connect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_SMD:
		gsmd_connect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_CHAR_BRIDGE:
		gbridge_connect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ret = ghsic_ctrl_connect(&gser->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_ctrl_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		ret = ghsic_data_connect(&gser->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_data_connect failed: err:%d\n",
					__func__, ret);
			ghsic_ctrl_disconnect(&gser->port, port_num);
			return ret;
		}
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static int gport_disconnect(struct f_gser *gser)
{
	unsigned port_num;

	port_num = gserial_ports[gser->port_num].client_port_num;

	pr_debug("%s: transport: %s f_gser: %pK gserial: %pK port_num: %d\n",
			__func__, xport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	switch (gser->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_disconnect(&gser->port);
		break;
	case USB_GADGET_XPORT_SMD:
		gsmd_disconnect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_CHAR_BRIDGE:
		gbridge_disconnect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_ctrl_disconnect(&gser->port, port_num);
		ghsic_data_disconnect(&gser->port, port_num);
		break;
	default:
		pr_err("%s: Un-supported transport:%s\n", __func__,
				xport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static void gser_complete_set_line_coding(struct usb_ep *ep,
		struct usb_request *req)
{
	struct f_gser            *gser = ep->driver_data;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;

	if (req->status != 0) {
		dev_dbg(&cdev->gadget->dev, "gser ttyGS%d completion, err %d\n",
			gser->port_num, req->status);
		return;
	}

	/* normal completion */
	if (req->actual != sizeof(gser->port_line_coding)) {
		dev_dbg(&cdev->gadget->dev, "gser ttyGS%d short resp, len %d\n",
			gser->port_num, req->actual);
		usb_ep_set_halt(ep);
	} else {
		struct usb_cdc_line_coding	*value = req->buf;
		gser->port_line_coding = *value;
	}
}
/*-------------------------------------------------------------------------*/

static int gser_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_gser		*gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	/* we know alt == 0, so this is an activation or a reset */

	if (gser->port.in->enabled) {
		dev_dbg(&cdev->gadget->dev,
			"reset generic ttyGS%d\n", gser->port_num);
		gserial_disconnect(&gser->port);
	}
	if (!gser->port.in->desc || !gser->port.out->desc) {
		dev_dbg(&cdev->gadget->dev,
			"activate generic ttyGS%d\n", gser->port_num);
		if (config_ep_by_speed(cdev->gadget, f, gser->port.in) ||
		    config_ep_by_speed(cdev->gadget, f, gser->port.out)) {
			gser->port.in->desc = NULL;
			gser->port.out->desc = NULL;
			return -EINVAL;
		}
	}
	gserial_connect(&gser->port, gser->port_num);
	return 0;
}

static void gser_disable(struct usb_function *f)
{
	struct f_gser	*gser = func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	dev_dbg(&cdev->gadget->dev,
		"generic ttyGS%d deactivated\n", gser->port_num);
	gserial_disconnect(&gser->port);
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int gser_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser		*gser = func_to_gser(f);
	int			status;
	struct usb_ep		*ep;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	
	if (gser_string_defs[0].id == 0 &&
		(gserial_ports[gser->port_num].serial_type == USB_FSER_FUNC_AUTOBOT ||
		gserial_ports[gser->port_num].serial_type == USB_FSER_FUNC_SERIAL)) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_string_defs[0].id = status;
	}

	if (modem_string_defs[0].id == 0 &&
		gserial_ports[gser->port_num].serial_type == USB_FSER_FUNC_MODEM) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		modem_string_defs[0].id = status;
	}

	if (modem_string_defs[1].id == 0 &&
		gserial_ports[gser->port_num].serial_type == USB_FSER_FUNC_MODEM_MDM) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		modem_string_defs[1].id = status;
	}

	switch (gserial_ports[gser->port_num].serial_type) {
	case USB_FSER_FUNC_MODEM:
		gser->port.func.name = "modem";
		gser->port.func.strings = modem_strings;
		gser_interface_desc.iInterface = modem_string_defs[0].id;
		break;
	case USB_FSER_FUNC_MODEM_MDM:
		gser->port.func.name = "modem_mdm";
		gser->port.func.strings = modem_strings;
		gser_interface_desc.iInterface = modem_string_defs[1].id;
		break;
	case USB_FSER_FUNC_AUTOBOT:
	case USB_FSER_FUNC_SERIAL:
		gser->port.func.name = "serial";
		gser->port.func.strings = gser_strings;
		gser_interface_desc.iInterface = gser_string_defs[0].id;
		break;
	default:
		break;
	}

	
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;
	gser_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &gser_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	gser_hs_in_desc.bEndpointAddress = gser_fs_in_desc.bEndpointAddress;
	gser_hs_out_desc.bEndpointAddress = gser_fs_out_desc.bEndpointAddress;

	gser_ss_in_desc.bEndpointAddress = gser_fs_in_desc.bEndpointAddress;
	gser_ss_out_desc.bEndpointAddress = gser_fs_out_desc.bEndpointAddress;

	status = usb_assign_descriptors(f, gser_fs_function, gser_hs_function,
			gser_ss_function);
	if (status)
		goto fail;
	dev_dbg(&cdev->gadget->dev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
		gser->port_num,
		gadget_is_superspeed(c->cdev->gadget) ? "super" :
		gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
		gser->port.in->name, gser->port.out->name);
	return 0;

fail:
	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static inline struct f_serial_opts *to_f_serial_opts(struct config_item *item)
{
	return container_of(to_config_group(item), struct f_serial_opts,
			    func_inst.group);
}

static void serial_attr_release(struct config_item *item)
{
	struct f_serial_opts *opts = to_f_serial_opts(item);

	usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations serial_item_ops = {
	.release	= serial_attr_release,
};

static ssize_t f_serial_port_num_show(struct config_item *item, char *page)
{
	return sprintf(page, "%u\n", to_f_serial_opts(item)->port_num);
}

CONFIGFS_ATTR_RO(f_serial_, port_num);

static struct configfs_attribute *acm_attrs[] = {
	&f_serial_attr_port_num,
	NULL,
};

static struct config_item_type serial_func_type = {
	.ct_item_ops	= &serial_item_ops,
	.ct_attrs	= acm_attrs,
	.ct_owner	= THIS_MODULE,
};

static void gser_free_inst(struct usb_function_instance *f)
{
	struct f_serial_opts *opts;

	opts = container_of(f, struct f_serial_opts, func_inst);
	gserial_free_line(opts->port_num);
	kfree(opts);
}

static struct usb_function_instance *gser_alloc_inst(void)
{
	struct f_serial_opts *opts;
	int ret;

	opts = kzalloc(sizeof(*opts), GFP_KERNEL);
	if (!opts)
		return ERR_PTR(-ENOMEM);

	opts->func_inst.free_func_inst = gser_free_inst;
	ret = gserial_alloc_line(&opts->port_num);
	if (ret) {
		kfree(opts);
		return ERR_PTR(ret);
	}
	config_group_init_type_name(&opts->func_inst.group, "",
				    &serial_func_type);

	return &opts->func_inst;
}

static void gser_free(struct usb_function *f)
{
	struct f_gser *serial;

	serial = func_to_gser(f);
	kfree(serial);
}

static void gser_unbind(struct usb_configuration *c, struct usb_function *f)
{
	usb_free_all_descriptors(f);
}

static struct usb_function *gser_alloc(struct usb_function_instance *fi)
{
	struct f_gser	*gser;
	struct f_serial_opts *opts;

	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof(*gser), GFP_KERNEL);
	if (!gser)
		return ERR_PTR(-ENOMEM);

	opts = container_of(fi, struct f_serial_opts, func_inst);

	gser->port_num = opts->port_num;

	gser->port.func.name = "gser";
	gser->port.func.strings = gser_strings;
	gser->port.func.bind = gser_bind;
	gser->port.func.unbind = gser_unbind;
	gser->port.func.set_alt = gser_set_alt;
	gser->port.func.disable = gser_disable;
	gser->port.func.free_func = gser_free;

	return &gser->port.func;
}

DECLARE_USB_FUNCTION_INIT(gser, gser_alloc_inst, gser_alloc);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Al Borchers");
MODULE_AUTHOR("David Brownell");

extern enum fserial_func_type serial_str_to_func_type(const char *name); 
int gserial_init_port(int port_num, const char *name,
		const char *port_name)
{
	enum transport_type transport;
	enum fserial_func_type serial_type; 
	int ret = 0;

	if (port_num >= GSERIAL_NO_PORTS)
		return -ENODEV;

	transport = str_to_xport(name);
	serial_type = serial_str_to_func_type(port_name); 
	pr_debug("%s, port:%d, transport:%s\n", __func__,
			port_num, xport_to_str(transport));

	gserial_ports[port_num].transport = transport;
	gserial_ports[port_num].port_num = port_num;
	gserial_ports[port_num].serial_type = serial_type; 

	switch (transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_ports[port_num].client_port_num = no_tty_ports; 
		no_tty_ports++;
		break;
	case USB_GADGET_XPORT_SMD:
		gserial_ports[port_num].client_port_num = no_smd_ports;
		no_smd_ports++;
		break;
	case USB_GADGET_XPORT_CHAR_BRIDGE:
		no_char_bridge_ports++;
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_ctrl_set_port_name(port_name, name);
		ghsic_data_set_port_name(port_name, name);

		/*client port number will be updated in gport_setup*/
		no_hsic_sports++;
		break;
	default:
		pr_err("%s: Un-supported transport transport: %u\n",
				__func__, gserial_ports[port_num].transport);
		return -ENODEV;
	}

	nr_ports++;

	return ret;
}

void gserial_deinit_port(void)
{
	no_char_bridge_ports = 0;
	no_tty_ports = 0;
	no_smd_ports = 0;
	no_hsic_sports = 0;
	nr_ports = 0;
}

bool gserial_is_connected(void)
{
	if (gserial_ports[0].gser_ptr != NULL)
		return gserial_ports[0].gser_ptr->online;
	return 0;
}

bool gserial_is_dun_w_softap_enabled(void)
{
	if (gserial_ports[0].gser_ptr != NULL)
		return gserial_ports[0].dun_w_softap_enable;
	return 0;
}

void gserial_dun_w_softap_enable(bool enable)
{
	pr_debug("android_usb: Setting dun_w_softap_enable to %u.",
		enable);

	gserial_ports[0].dun_w_softap_enable = enable;
}

bool gserial_is_dun_w_softap_active(void)
{
	if (gserial_ports[0].gser_ptr != NULL)
		return gserial_ports[0].dun_w_softap_enable &&
			gserial_ports[0].gser_ptr->online;
	return 0;
}

static inline int gser_device_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -EBUSY;
	}
}

static inline void gser_device_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static int gser_open_dev(struct inode *ip, struct file *fp)
{
	struct f_gser *gser = gserial_ports[0].gser_ptr;

	pr_debug("%s: Open serial device", __func__);

	if (!gser) {
		pr_err("%s: Serial device not created yet", __func__);
		return -ENODEV;
	}

	if (gser_device_lock(&gser->open_excl)) {
		pr_err("%s: Already opened", __func__);
		return -EBUSY;
	}

	fp->private_data = gser;
	pr_debug("%s: Serial device opened", __func__);

	return 0;
}

static int gser_release_dev(struct inode *ip, struct file *fp)
{
	struct f_gser *gser = fp->private_data;

	pr_debug("%s: Close serial device", __func__);

	if (!gser) {
		pr_err("Serial device not created yet\n");
		return -ENODEV;
	}

	gser_device_unlock(&gser->open_excl);

	return 0;
}

static void gser_ioctl_set_transport(struct f_gser *gser,
	unsigned int transport)
{
	int ret;
	enum transport_type new_transport;
	const struct usb_endpoint_descriptor *ep_in_desc_backup;
	const struct usb_endpoint_descriptor *ep_out_desc_backup;

	if (transport == GSERIAL_SET_XPORT_TYPE_TTY) {
		new_transport = USB_GADGET_XPORT_TTY;
		pr_debug("%s: Switching modem transport to TTY.", __func__);
		gser->port.flags |= ASYNC_LOW_LATENCY;
	} else if (transport == GSERIAL_SET_XPORT_TYPE_SMD) {
		new_transport = USB_GADGET_XPORT_SMD;
		pr_debug("%s: Switching modem transport to SMD.", __func__);
	} else {
		pr_err("%s: Wrong transport type %d", __func__, transport);
		return;
	}

	if (gser->transport == new_transport) {
		pr_debug("%s: Modem transport aready set to this type.",
			__func__);
		return;
	}

	ep_in_desc_backup  = gser->port.in->desc;
	ep_out_desc_backup = gser->port.out->desc;
	gport_disconnect(gser);
	if (new_transport == USB_GADGET_XPORT_TTY) {
		ret = gserial_alloc_line(
			&gserial_ports[gser->port_num].client_port_num);
		if (ret)
			pr_debug("%s: Unable to alloc TTY line", __func__);
	}

	gser->port.in->desc  = ep_in_desc_backup;
	gser->port.out->desc = ep_out_desc_backup;
	gser->transport = new_transport;
	gport_connect(gser);
	pr_debug("%s: Modem transport switch is complete.", __func__);

}

static long gser_ioctl(struct file *fp, unsigned cmd, unsigned long arg)
{
	int ret = 0;
	int count;
	int xport_type;
	int smd_port_num;
	char smd_write_buf[GSERIAL_BUF_LEN];
	struct ioctl_smd_write_arg_type smd_write_arg;
	struct f_gser *gser;
	void __user *argp = (void __user *)arg;

	if (!fp || !fp->private_data) {
		pr_err("%s: Invalid file handle", __func__);
		return -EBADFD;
	}

	gser = fp->private_data;
	pr_debug("Received command %d", cmd);

	if (gser_device_lock(&gser->ioctl_excl))
		return -EBUSY;

	switch (cmd) {
	case GSERIAL_SET_XPORT_TYPE:
		if (copy_from_user(&xport_type, argp, sizeof(xport_type))) {
			pr_err("%s: failed to copy IOCTL set transport type",
				__func__);
			ret = -EFAULT;
			break;
		}

		gser_ioctl_set_transport(gser, xport_type);
		break;

	case GSERIAL_SMD_WRITE:
		if (gser->transport != USB_GADGET_XPORT_SMD) {
			pr_err("%s: ERR: Got SMD WR cmd when not in SMD mode",
				__func__);

			break;
		}

		pr_debug("%s: Copy GSERIAL_SMD_WRITE IOCTL command argument",
			__func__);
		if (copy_from_user(&smd_write_arg, argp,
			sizeof(smd_write_arg))) {
			ret = -EFAULT;

			pr_err("%s: failed to copy IOCTL GSERIAL_SMD_WRITE arg",
				__func__);

			break;
		}
		smd_port_num =
			gserial_ports[gser->port_num].client_port_num;

		if (smd_write_arg.size > GSERIAL_BUF_LEN) {
			pr_err("%s: Invalid size:%u, max: %u", __func__,
				smd_write_arg.size, GSERIAL_BUF_LEN);
			ret = -EINVAL;
			break;
		}

		pr_debug("%s: Copying %d bytes from user buffer to local\n",
			__func__, smd_write_arg.size);

		if (copy_from_user(smd_write_buf, smd_write_arg.buf,
			smd_write_arg.size)) {

			pr_err("%s: failed to copy buf for GSERIAL_SMD_WRITE",
				__func__);

			ret = -EFAULT;
			break;
		}

		pr_debug("%s: Writing %d bytes to SMD channel\n",
			__func__, smd_write_arg.size);

		count = gsmd_write(smd_port_num, smd_write_buf,
			smd_write_arg.size);

		if (count != smd_write_arg.size)
			ret = -EFAULT;

		break;
	default:
		pr_err("Unsupported IOCTL");
		ret = -EINVAL;
	}

	gser_device_unlock(&gser->ioctl_excl);
	return ret;
}