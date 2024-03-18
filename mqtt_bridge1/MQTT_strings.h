#include <map>
#include <vector>
#include <list>


static const char* root_ca_yandex PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFGTCCAwGgAwIBAgIQJMM7ZIy2SYxCBgK7WcFwnjANBgkqhkiG9w0BAQ0FADAf
MR0wGwYDVQQDExRZYW5kZXhJbnRlcm5hbFJvb3RDQTAeFw0xMzAyMTExMzQxNDNa
Fw0zMzAyMTExMzUxNDJaMB8xHTAbBgNVBAMTFFlhbmRleEludGVybmFsUm9vdENB
MIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAgb4xoQjBQ7oEFk8EHVGy
1pDEmPWw0Wgw5nX9RM7LL2xQWyUuEq+Lf9Dgh+O725aZ9+SO2oEs47DHHt81/fne
5N6xOftRrCpy8hGtUR/A3bvjnQgjs+zdXvcO9cTuuzzPTFSts/iZATZsAruiepMx
SGj9S1fGwvYws/yiXWNoNBz4Tu1Tlp0g+5fp/ADjnxc6DqNk6w01mJRDbx+6rlBO
aIH2tQmJXDVoFdrhmBK9qOfjxWlIYGy83TnrvdXwi5mKTMtpEREMgyNLX75UjpvO
NkZgBvEXPQq+g91wBGsWIE2sYlguXiBniQgAJOyRuSdTxcJoG8tZkLDPRi5RouWY
gxXr13edn1TRDGco2hkdtSUBlajBMSvAq+H0hkslzWD/R+BXkn9dh0/DFnxVt4XU
5JbFyd/sKV/rF4Vygfw9ssh1ZIWdqkfZ2QXOZ2gH4AEeoN/9vEfUPwqPVzL0XEZK
r4s2WjU9mE5tHrVsQOZ80wnvYHYi2JHbl0hr5ghs4RIyJwx6LEEnj2tzMFec4f7o
dQeSsZpgRJmpvpAfRTxhIRjZBrKxnMytedAkUPguBQwjVCn7+EaKiJfpu42JG8Mm
+/dHi+Q9Tc+0tX5pKOIpQMlMxMHw8MfPmUjC3AAd9lsmCtuybYoeN2IRdbzzchJ8
l1ZuoI3gH7pcIeElfVSqSBkCAwEAAaNRME8wCwYDVR0PBAQDAgGGMA8GA1UdEwEB
/wQFMAMBAf8wHQYDVR0OBBYEFKu5xf+h7+ZTHTM5IoTRdtQ3Ti1qMBAGCSsGAQQB
gjcVAQQDAgEAMA0GCSqGSIb3DQEBDQUAA4ICAQAVpyJ1qLjqRLC34F1UXkC3vxpO
nV6WgzpzA+DUNog4Y6RhTnh0Bsir+I+FTl0zFCm7JpT/3NP9VjfEitMkHehmHhQK
c7cIBZSF62K477OTvLz+9ku2O/bGTtYv9fAvR4BmzFfyPDoAKOjJSghD1p/7El+1
eSjvcUBzLnBUtxO/iYXRNo7B3+1qo4F5Hz7rPRLI0UWW/0UAfVCO2fFtyF6C1iEY
/q0Ldbf3YIaMkf2WgGhnX9yH/8OiIij2r0LVNHS811apyycjep8y/NkG4q1Z9jEi
VEX3P6NEL8dWtXQlvlNGMcfDT3lmB+tS32CPEUwce/Ble646rukbERRwFfxXojpf
C6ium+LtJc7qnK6ygnYF4D6mz4H+3WaxJd1S1hGQxOb/3WVw63tZFnN62F6/nc5g
6T44Yb7ND6y3nVcygLpbQsws6HsjX65CoSjrrPn0YhKxNBscF7M7tLTW/5LK9uhk
yjRCkJ0YagpeLxfV1l1ZJZaTPZvY9+ylHnWHhzlq0FzcrooSSsp4i44DB2K7O2ID
87leymZkKUY6PMDa4GkDJx0dG4UXDhRETMf+NkYgtLJ+UIzMNskwVDcxO4kVL+Hi
Pj78bnC5yCw8P5YylR45LdxLzLO68unoXOyFz1etGXzszw8lJI9LNubYxk77mK8H
LpuQKbSbIERsmR+QqQ==
-----END CERTIFICATE-----
)EOF";


static const char* root_ca_hivemq PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


namespace MQTTLanguage {

struct Texts {
    const char
        *error_mqtt,
        *online,
        *error_driver,
        *error_module,
        *error_timeout,
        *error_wifi,
        *error_format,
        *config_wifi,
        // *config_ota,
        *config_mqtt,
        *config_mqtt_server,
        *config_mqtt_port,
        *config_max_failures,
        *config_mqtt_topic,
        *config_Submqtt_topic,
        *config_mqtt_interval,
        *config_mqtt_template,
        *config_template_info,
        *connecting,
        *wait
    ;
    std::vector<std::list<String>> portal_instructions;
    std::list<String> DispSubMsg;
    std::list<String> first_run;
    std::list<String> calibration;
    std::list<String> calibrating;
};

std::map<const String, const String> languages {
    // Ordered alphabetically
    { "en", "English" },
    { "nl", "Nederlands" },
};

bool available(const String& language) {
    return languages.count(language) == 1;
}

bool select(Texts& T, String language) {
    if (! available(language)) {
        if (available("en")) language = "en";
        else language = languages.begin()->first;
    }

    if (language == "en") {
        T.error_mqtt = "MQTT unreachable";
        T.online = "still online";
        T.error_driver = "driver error";
        T.error_module = "module turned around!";
        T.error_timeout = "Time's up";
        T.error_wifi = "WiFi failed!";
        T.error_format = "Formatting failed";
        T.wait = "wait...";
        T.config_wifi = "Use WiFi connection";
        // T.config_ota = "Enable wireless reprogramming. (Uses portal password!)";
        T.config_mqtt = "Publish  via the MQTT protocol";
        T.config_mqtt_server = "Broker";  // probably should not be translated
        T.config_mqtt_port = "Broker TCP port";
        T.config_max_failures = "Number of failed connections before automatic restart";
        T.config_mqtt_topic = "Publish Topic";  // probably should not be translated
        T.config_Submqtt_topic = "Subscribe Topic";  // probably should not be translated
        T.config_mqtt_interval = "Keep alive publish interval [s]";
        T.config_mqtt_template = "Payload template";
        T.config_template_info = "The {} in the template is replaced by the hostname.";
        T.connecting = "Connecting to WiFi...";
        T.portal_instructions = {
            {
                "For configuration,",
                "connect to WiFi",
                "\"{ssid}\"",
                "with a smartphone."
            },
            {
                "Follow instructions",
                "on your smartphone.",
                "(log in notification)"
            },
            {
                "Change settings",
                "and click \"Save\".",
                "(bottom right)"
            },
            {
                "Change settings",
                "and click \"Save\".",
                "Or \"Restart device\"",
                "when you're done."
            }
        };
        T.DispSubMsg = {
            "Incoming",
            "\"{topic}\"",
            "\"{payload}\"",
        };
        T.first_run = {
            "DO NOT TURN OFF",
            "Initializing",
            "flash memory.",
        };
        T.calibration = {
            "Manual calibration!",
            "Press button",
            "to cancel.",
            ""
        };
        T.calibrating = {
            "Assuming current",
            "CO2 level to be",
            "400 PPM."
        };
        return true;
    }

    if(language == "nl") {
        T.error_mqtt = "MQTT onbereikbaar";
        T.online = "nog online";
        T.error_driver = "driverfout";
        T.error_module = "module verkeerd om!";
        T.error_timeout = "Tijd verstreken";
        T.error_wifi = "WiFi mislukt!";
        T.error_format = "Formatteren mislukt";
        T.wait = "wacht...";
        T.config_wifi = "WiFi-verbinding gebruiken";
        // T.config_ota = "Draadloos herprogrammeren inschakelen. (Gebruikt portaalwachtwoord!)";
        T.config_mqtt = "Data via het MQTT-protocol versturen";
        T.config_mqtt_server = "Broker";  
        T.config_mqtt_port = "Broker TCP-poort";
        T.config_max_failures = "Aantal verbindingsfouten voor automatische herstart";
        T.config_mqtt_topic = "Publish Topic";  // probably should not be translated
        T.config_Submqtt_topic = "Subscribe Topic";  // probably should not be translated
        T.config_mqtt_interval = "Keep alive Publicatie-interval [s]";
        T.config_mqtt_template = "Payload sjabloon";
        T.config_template_info = "De {} in het sjabloon wordt vervangen door de hostname ";
        T.connecting = "Verbinden met WiFi...";
        T.portal_instructions = {
            {
                "Voor configuratie,",
                "verbind met WiFi",
                "\"{ssid}\"",
                "met een smartphone."
            },
            {
                "Volg instructies op",
                "uw smartphone.",
                "(inlog-notificatie)"
            },
            {
                "Wijzig instellingen",
                "en klik op \"Opslaan\".",
                "(rechtsonder)"
            },
            {
                "Wijzig instellingen",
                "en klik op \"Opslaan\".",
                "Of \"Herstarten\"",
                "als u klaar bent."
            }
        };
        T.DispSubMsg = {
            "Incoming",
            "\"{topic}\"",
            "\"{payload}\"",
        };
        T.first_run = {
            "NIET",
            "UITSCHAKELEN",
            "Flashgeheugen",
            "wordt voorbereid."
        };
        T.calibration = {
            "Handmatige",
            "calibratie!",
            "knop = stop",
            ""
        };
        T.calibrating = {
            "Het huidige CO2-",
            "niveau wordt",
            "aangenomen",
            "400 PPM te zijn."
        };
        return true;
    }

    return false;
}

} // namespace
