#include <Arduino.h>

// Define error codes
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_ERR_NOT_SUPPORTED -3

#define PHY_CHECK(a, str, goto_tag, ...)                                          \
    do                                                                            \
    {                                                                             \
        if (!(a))                                                                 \
        {                                                                         \
            Serial.printf("%s(%d): " str "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                        \
        }                                                                         \
    } while (0)

// Define the registers as structs for easier access
typedef union {
    struct {
        uint32_t reserved_7_0 : 8;  // Reserved
        uint32_t pdpxmd : 1;        // PHY Duplex Mode bit
        uint32_t reserved_10_9: 2;  // Reserved
        uint32_t ppwrsv: 1;         // PHY Power-Down bit
        uint32_t reserved_13_12: 2; // Reserved
        uint32_t ploopbk: 1;        // PHY Loopback bit
        uint32_t prst: 1;           // PHY Software Reset bit
    };
    uint32_t val;
} phcon1_reg_t;

#define ETH_PHY_PHCON1_REG_ADDR (0x00)

typedef union {
    struct {
        uint32_t reserved_7_0 : 8;  // Reserved
        uint32_t hdldis : 1;        // Half-Duplex Loopback Disable
        uint32_t reserved_9: 1;     // Reserved
        uint32_t jabber: 1;         // Disable Jabber Correction
        uint32_t reserved_12_11: 2; // Reserved
        uint32_t txdis: 1;          // Disable Twist-Pair Transmitter
        uint32_t frclnk: 1;         // Force Linkup
        uint32_t reserved_15: 1;    // Reserved
    };
    uint32_t val;
} phcon2_reg_t;

#define ETH_PHY_PHCON2_REG_ADDR (0x10)

typedef union {
    struct {
        uint32_t reserved_4_0 : 5;   // Reserved
        uint32_t plrity : 1;         // Polarity Status
        uint32_t reserved_8_6 : 3;   // Reserved
        uint32_t dpxstat : 1;        // PHY Duplex Status
        uint32_t lstat : 1;          // PHY Link Status (non-latching)
        uint32_t colstat : 1;        // PHY Collision Status
        uint32_t rxstat : 1;         // PHY Receive Status
        uint32_t txstat : 1;         // PHY Transmit Status
        uint32_t reserved_15_14 : 2; // Reserved
    };
    uint32_t val;
} phstat2_reg_t;

#define ETH_PHY_PHSTAT2_REG_ADDR (0x11)

typedef struct {
    void (*reset)();
    void (*reset_hw)();
    void (*init)();
    void (*deinit)();
    void (*set_mediator)(void*);
    void (*autonego_ctrl)(int, bool*);
    void (*get_link)();
    void (*pwrctl)(bool);
    void (*get_addr)(uint32_t*);
    void (*set_addr)(uint32_t);
    void (*set_speed)(int);
    void (*set_duplex)(int);
    void (*del)();
} esp_eth_phy_t;

typedef struct {
    esp_eth_phy_t parent;
    void* eth;  // Assuming you have some type for the mediator
    uint32_t addr;
    uint32_t reset_timeout_ms;
    int link_status;
    int reset_gpio_num;
} phy_enc28j60_t;

esp_err_t enc28j60_update_link_duplex_speed(phy_enc28j60_t *enc28j60) {
    void* eth = enc28j60->eth;
    int speed = 10;  // enc28j60 speed is fixed to 10Mbps
    int duplex = 0;  // ETH_DUPLEX_HALF
    phstat2_reg_t phstat;
    // Read the register (you need to implement this)
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_PHSTAT2_REG_ADDR, &(phstat.val))) {
    //     return ESP_FAIL;
    // }
    int link = phstat.lstat ? 1 : 0; // ETH_LINK_UP : ETH_LINK_DOWN
    if (enc28j60->link_status != link) {
        if (link == 1) { // ETH_LINK_UP
            duplex = phstat.dpxstat ? 1 : 0; // ETH_DUPLEX_FULL or ETH_DUPLEX_HALF
            // Implement the state change callbacks
            // if (!eth->on_state_changed(eth, 1, (void*)speed)) return ESP_FAIL; // ETH_STATE_SPEED
            // if (!eth->on_state_changed(eth, 2, (void*)duplex)) return ESP_FAIL; // ETH_STATE_DUPLEX
        }
        // if (!eth->on_state_changed(eth, 0, (void*)link)) return ESP_FAIL; // ETH_STATE_LINK
        enc28j60->link_status = link;
    }
    return ESP_OK;
}

esp_err_t enc28j60_set_mediator(esp_eth_phy_t *phy, void* eth) {
    if (!eth) return ESP_ERR_INVALID_ARG;
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    enc28j60->eth = eth;
    return ESP_OK;
}

esp_err_t enc28j60_get_link(esp_eth_phy_t *phy) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    if (enc28j60_update_link_duplex_speed(enc28j60) != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t enc28j60_reset(esp_eth_phy_t *phy) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    enc28j60->link_status = 0; // ETH_LINK_DOWN
    void* eth = enc28j60->eth;
    phcon1_reg_t phcon1 = { .val = 0x8000 }; // Reset bit
    // Implement register write
    // if (!eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, phcon1.val)) return ESP_FAIL;

    uint32_t to = 0;
    for (to = 0; to < enc28j60->reset_timeout_ms / 10; to++) {
        delay(10);
        // Read register to check reset status
        // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, &(phcon1.val))) return ESP_FAIL;
        if (!(phcon1.val & 0x8000)) break;
    }
    if (to >= enc28j60->reset_timeout_ms / 10) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t enc28j60_reset_hw(esp_eth_phy_t *phy) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    if (enc28j60->reset_gpio_num >= 0) {
        pinMode(enc28j60->reset_gpio_num, OUTPUT);
        digitalWrite(enc28j60->reset_gpio_num, LOW);
        delay(10); // A short delay
        digitalWrite(enc28j60->reset_gpio_num, HIGH);
    }
    return ESP_OK;
}

esp_err_t enc28j60_autonego_ctrl(esp_eth_phy_t *phy, int cmd, bool *autoneg_en_stat) {
    switch (cmd) {
    case 1: // ESP_ETH_PHY_AUTONEGO_RESTART
    case 2: // ESP_ETH_PHY_AUTONEGO_EN
        return ESP_ERR_NOT_SUPPORTED;
    case 3: // ESP_ETH_PHY_AUTONEGO_DIS
    case 4: // ESP_ETH_PHY_AUTONEGO_G_STAT
        *autoneg_en_stat = false;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t enc28j60_set_speed(esp_eth_phy_t *phy, int speed) {
    if (speed == 10) { // ETH_SPEED_10M
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t enc28j60_set_duplex(esp_eth_phy_t *phy, int duplex) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    void* eth = enc28j60->eth;
    phcon1_reg_t phcon1;

    enc28j60->link_status = 0; // ETH_LINK_DOWN

    // Read register
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_PHCON1_REG_ADDR, &(phcon1.val))) return ESP_FAIL;

    switch (duplex) {
    case 0: // ETH_DUPLEX_HALF
        phcon1.pdpxmd = 0;
        break;
    case 1: // ETH_DUPLEX_FULL
        phcon1.pdpxmd = 1;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    // Write register
    // if (!eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_PHCON1_REG_ADDR, phcon1.val)) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t enc28j60_pwrctl(esp_eth_phy_t *phy, bool enable) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    void* eth = enc28j60->eth;
    phcon1_reg_t phcon1;
    // Read register
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, &(phcon1.val))) return ESP_FAIL;

    if (!enable) {
        phcon1.val |= (1 << 11); // Set power down bit
    } else {
        phcon1.val &= ~(1 << 11); // Clear power down bit
    }
    // Write register
    // if (!eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, phcon1.val)) return ESP_FAIL;
    // Read register again to verify
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_BMCR_REG_ADDR, &(phcon1.val))) return ESP_FAIL;

    if (!enable && (phcon1.val & (1 << 11))) return ESP_FAIL;
    if (enable && !(phcon1.val & (1 << 11))) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t enc28j60_set_addr(esp_eth_phy_t *phy, uint32_t addr) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    enc28j60->addr = addr;
    return ESP_OK;
}

esp_err_t enc28j60_get_addr(esp_eth_phy_t *phy, uint32_t *addr) {
    if (!addr) return ESP_ERR_INVALID_ARG;
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    *addr = enc28j60->addr;
    return ESP_OK;
}

esp_err_t enc28j60_del(esp_eth_phy_t *phy) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    free(enc28j60);
    return ESP_OK;
}

esp_err_t enc28j60_init(esp_eth_phy_t *phy) {
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)phy;
    void* eth = enc28j60->eth;
    if (enc28j60_pwrctl(phy, true) != ESP_OK) return ESP_FAIL;
    if (enc28j60_reset(phy) != ESP_OK) return ESP_FAIL;

    // Read PHY ID
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_IDR1_REG_ADDR, &(id1.val))) return ESP_FAIL;
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_IDR2_REG_ADDR, &(id2.val))) return ESP_FAIL;
    // if (id1.oui_msb != 0x0083 || id2.oui_lsb != 0x05 || id2.vendor_model != 0x00) return ESP_FAIL;

    // Disable half duplex loopback
    phcon2_reg_t phcon2;
    // if (!eth->phy_reg_read(eth, enc28j60->addr, ETH_PHY_PHCON2_REG_ADDR, &(phcon2.val))) return ESP_FAIL;
    phcon2.hdldis = 1;
    // if (!eth->phy_reg_write(eth, enc28j60->addr, ETH_PHY_PHCON2_REG_ADDR, phcon2.val)) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t enc28j60_deinit(esp_eth_phy_t *phy) {
    if (enc28j60_pwrctl(phy, false) != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

esp_eth_phy_t* esp_eth_phy_new_enc28j60(const void *config) {
    if (!config) return NULL;
    phy_enc28j60_t *enc28j60 = (phy_enc28j60_t*)calloc(1, sizeof(phy_enc28j60_t));
    if (!enc28j60) return NULL;

    enc28j60->addr = ((const eth_phy_config_t*)config)->phy_addr;
    enc28j60->reset_timeout_ms = ((const eth_phy_config_t*)config)->reset_timeout_ms;
    enc28j60->reset_gpio_num = ((const eth_phy_config_t*)config)->reset_gpio_num;
    enc28j60->link_status = 0; // ETH_LINK_DOWN
    enc28j60->parent.reset = enc28j60_reset;
    enc28j60->parent.reset_hw = enc28j60_reset_hw;
    enc28j60->parent.init = enc28j60_init;
    enc28j60->parent.deinit = enc28j60_deinit;
    enc28j60->parent.set_mediator = enc28j60_set_mediator;
    enc28j60->parent.autonego_ctrl = enc28j60_autonego_ctrl;
    enc28j60->parent.get_link = enc28j60_get_link;
    enc28j60->parent.pwrctl = enc28j60_pwrctl;
    enc28j60->parent.get_addr = enc28j60_get_addr;
    enc28j60->parent.set_addr = enc28j60_set_addr;
    enc28j60->parent.set_speed = enc28j60_set_speed;
    enc28j60->parent.set_duplex = enc28j60_set_duplex;
    enc28j60->parent.del = enc28j60_del;
    return &(enc28j60->parent);
}
