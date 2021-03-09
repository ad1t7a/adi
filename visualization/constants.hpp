#pragma once

#include <string>

namespace adi {
namespace visualization {
//! visualizer path
static std::string kVisualizerPath = "/adi/";

//! correct not utf-8 to utf-8
inline std::string correct_non_utf_8(const std::string &str) {
  int i, f_size = str.size();
  unsigned char c, c2, c3, c4;
  std::string to;
  to.reserve(f_size);

  for (i = 0; i < f_size; i++) {
    c = (unsigned char)(str)[i];
    if (c < 32) {                         // control char
      if (c == 9 || c == 10 || c == 13) { // allow only \t \n \r
        to.append(1, c);
      }
      continue;
    } else if (c < 127) { // normal ASCII
      to.append(1, c);
      continue;
    } else if (c < 160) { // control char (nothing should be defined here
                          // either ASCI, ISO_8859-1 or UTF8, so skipping)
      if (c2 == 128) {    // fix microsoft mess, add euro
        to.append(1, 226);
        to.append(1, 130);
        to.append(1, 172);
      }
      if (c2 == 133) { // fix IBM mess, add NEL = \n\r
        to.append(1, 10);
        to.append(1, 13);
      }
      continue;
    } else if (c < 192) { // invalid for UTF8, converting ASCII
      to.append(1, (unsigned char)194);
      to.append(1, c);
      continue;
    } else if (c < 194) { // invalid for UTF8, converting ASCII
      to.append(1, (unsigned char)195);
      to.append(1, c - 64);
      continue;
    } else if (c < 224 && i + 1 < f_size) { // possibly 2byte UTF8
      c2 = (unsigned char)(str)[i + 1];
      if (c2 > 127 && c2 < 192) {   // valid 2byte UTF8
        if (c == 194 && c2 < 160) { // control char, skipping
          ;
        } else {
          to.append(1, c);
          to.append(1, c2);
        }
        i++;
        continue;
      }
    } else if (c < 240 && i + 2 < f_size) { // possibly 3byte UTF8
      c2 = (unsigned char)(str)[i + 1];
      c3 = (unsigned char)(str)[i + 2];
      if (c2 > 127 && c2 < 192 && c3 > 127 && c3 < 192) { // valid 3byte UTF8
        to.append(1, c);
        to.append(1, c2);
        to.append(1, c3);
        i += 2;
        continue;
      }
    } else if (c < 245 && i + 3 < f_size) { // possibly 4byte UTF8
      c2 = (unsigned char)(str)[i + 1];
      c3 = (unsigned char)(str)[i + 2];
      c4 = (unsigned char)(str)[i + 3];
      if (c2 > 127 && c2 < 192 && c3 > 127 && c3 < 192 && c4 > 127 &&
          c4 < 192) { // valid 4byte UTF8
        to.append(1, c);
        to.append(1, c2);
        to.append(1, c3);
        to.append(1, c4);
        i += 3;
        continue;
      }
    }
    // invalid UTF8, converting ASCII (c>245 || string too short for
    // multi-byte))
    to.append(1, (unsigned char)195);
    to.append(1, c - 64);
  }
  return to;
}

//! texture data
static std::string kTextureDataBrokenRobot =
    "data:image/"
    "png;base64,iVBORw0KGgoAAAANSUhEUgAAAKsAAADVCAMAAAAfHvCaAAAAGFBMVEVYn%"
    "2BH%2F%2F%2F%2Bex%2B3U5vd7s%2Bfq8%"
    "2Fs0itq72PMLUPvtAAASvklEQVR4AbXBC0JqCQxEwT5Jd7L%2FHc8FdR4g%2BEGtEr8u%"
    "2FBHxu7otdzd%2FQPyqlmRp1Pw%"
    "2B8aukDfRa1fw28ZtWy4sa89vEb7LCi0zx28RvqgkvouW3id%2FU8pbtWmv5beJXRWNrRmp%"
    "2BnfhlHXZm%2BQPi95Vk%2FoD4fZbMHxC%2FryTzB8Tva435A%2BL3rcb8AfH7VjJ%2FQPy%"
    "2BHYk%2FIH5facwfEL8iaZcrnKyn%2BAPi57K2VL2WF1hJ%"
    "2FAHxQ2tJrg6HteXVjPkD4ge6V3J1%2BF97zhx%"
    "2BnXhWb8nacKXlnYPErxNPyfqw4ZYKVuUZdfhd4hmxunY73NICgfWMOvwm8ZQ1pMvlDZdaCi"
    "c98kjV4beIp8ScpLvsSvhflzqQmqVLB281v0E8pc2bdNne8EayNTPNSbt02PBj4intcKltb%"
    "2FNibY%2BLf9aSO%"
    "2FyMeMo6XMva3g0vwrWsxvyMeEoc3knZ2g53ZaXa8DzxlHa4J23Jae5aycXTxFPa4WRdXAtd"
    "sivckZXG4TniKWtOSlre6y7LG651Wxq5OzxDPGUVIKNwX6ekCv%"
    "2B0ddglVPMM8ZQ10FJ4LGVvOEuXRl7OqnmGeEor4Ck%2BtnI1ZEvjDa%"
    "2FcPEM8ZQVY4RO9VqUlN%2F84PEM8JQ50cUgXH2mrKlyq5RniOQ4vVjPLHdu86OKGi2eIr%"
    "2BgNV6JwljmYO6zlbJsbWp4hPtVrjYpLLV7UHIp7rOVkixtaniE%"
    "2BU5I2Nc2FKJytZhTuiac5rLnh4hniEzUbDjXhn3g5W0nNA1aAKm7YPEN8bMecrZYLWl70hk"
    "cyBay5YfMM8aHI4aR7xAUVHyirOdhAmRsqniE%2BtOKsRjIXtDzmmRGHVmDFDRfPEB%"
    "2BJzMmO01xScdYnVRs6vPHMFG9W4ZrMM8RHouWw43DNhlDWiSVZY3nDoWYc3qzDNZlniPe6w"
    "4uoOFjcKhPXuJNWyG6VqjSuhm7%"
    "2BiZorUfEM8U5J8nKyMw0tcZLwPxdRtTlUcUgVdGlml0uZ4pqKZ4hr5VUnpSXdUgVa4hA5vH"
    "ERV1Tp9XhdJTWHksYd%"
    "2Ftdarql4hrjQiaPiYLclNSeebVYz5o0W7Ghsa9blmlFtx01rxP8yy5XIPEP8L1W7bjWHlbz"
    "hRTwjzXrCK1f3qqSEyBysLVtayKp40yqurcITxJtUgavVHNob%"
    "2FinZTWt5VVvWVKvJSttQCkRjb%2FA4vLK5thOeIN6sm9ai5cTFhYRDy%"
    "2FyTGpdU0hxkaZvWUrZluTmLims14QniVbywClqgeouT9IZXNWoupGzNqHa3y5LGVYBnipbC"
    "SVxcq1meIN54oRXsbEk26S3NmBcZ807K3gon2ZLcxF5tPMVJprlWE54g3nihtbRHm7WjkbxT"
    "HSCWwj1r2U4HSMmdQEmWwonNtah4gnhjA9ZSaohmpnpDjWRptDwS25LcQGsc2Bla5sTFtZV4"
    "gnixpWmIVWpgRuVwsiV5q7kv0JJcNVIFapydUrHTQKa5IfMEcRKrurSQ0qhsmVR4kea%"
    "2B7pIr9NqSrRltWlaxomUgVVyLxBPEYeUGygtszew2KfOBclVpVN2ctCXNidZaaKWmONhc6r"
    "KaJwi6xuGkRmWpAkRa7outF9XN%2F7LlmbJmpiCyvBxk%"
    "2FtnSqHmGWGk5i2ZcaWBLau5KKHt3Ce%2FsaLMz46VG4cTFm%"
    "2FaMOzxFUYWztjzhkNI43JPyYvPAegPxzFRpOYmWF1WywrPUag5xjRapqqxxubijvYFVaC%"
    "2Fv7YSDpzxjzlbhpKXxhqcpWshqtECk0Yys6m5utZdD1LCuCifhfyVOapqsxhyiQMmSm58QN"
    "dZheZGV5FqwueXiZBUga28DvRte1NQCpQVSUkFqPbIr%"
    "2FIxg7arwJqqEg6e5Vuas1Zytyw1ka5uT9ajKI87WbksaLT8mbkXFyWqaa2rOVuFVStUNpGr"
    "DoSTPmDfWdlby8kPiHQtoa0vLpXU4WzX%"
    "2FS5W2gWxtOHQ24U3CSUmu8BPinR2XVSFyuNAOZ9Fyae1qDu2qcF8suRKeJt7pcW1zaE9xwc"
    "VZq7nWtpeTrQ0PrEeq8CTxnsWrlbThELra5ixqbsXWNoeq6nBft6TlOeK9VnG2lfb4TKOOlp"
    "OouKPsWg4pb3Nf1uMGusP3iDtKDaTcgMuWvL1FmZOouCtlbwJs1Yb7SuN2Nd8k7mgvXV4OKW"
    "ALiGkVJ14eyPqQQG9Vc0dWGnn5LnFPTW1z1gW0OdSyag5aHsvaroVs1YZL2dKMt1nzXeKula"
    "s52QLanGy3xq4a87Eu2yHZ2uZNWzPjDbDmu8R9a8m7iQNscbKyy%2BWS%"
    "2BUzWtqp7qzpA1jPj8KKK7xIPZG2NVWTTSbpKbs5cfEF6y64qV6ctqcKbdvgm8VhSlnWwJbu"
    "aV3LzRb11onFt%2BKcVvkl8one7u3bD%2FzJuXnRt%2BFTXVHOWqubQ4rvEEyI1L1Z2h8%"
    "2B0eRHLKiBqvkk8IePmxZq1lk%"
    "2B0w0nJUHKIlm8ST8ioeVEFtFwbPhA3h8gcdpZV803iCRkVL7Y42bK2w0NlDqXlpJRV803iG"
    "ZYrnFRxlqwO3eEuN4dSOGlVme8Sz7C37QZqeZPekl0b3nMBreKsp1bNN4lnWIEtF1Vc6i1bV"
    "ZtwxQX0NC9UrfBN4hk7zaHLNrey1kgVLljATnO2rmj5JvEMqzlrF%2B%"
    "2BFXitcsAArnFkdLd8knrFqPmFzyQq0xUm0tJZvEs8oAR0eix0u1ARSqg70NNHyTeIZUqgZ8"
    "5gdLlgcMjOSRlBqvkk8wwOSp3moJlyoCYfeKkmBVvgm8YyaUJJ5zOJSTXMWSgus%"
    "2BC7xjJpA%2BMiquVATXiUcSuGbxDNqmk%"
    "2BUxtW82WmurMI3iWd4wifaHo1rNxx2miul8E3iGTXhc4nH0lQ1O80VK3yTeEYNX5SspbEnX"
    "FmFbxLPqGm%2BrsvWFFdK4ZvEM2rCt6RmzCWL7xLP2Anfs2M3Fyy%"
    "2BSzyjpvmqDoed5YrFd4ln7DRftHI19BRXSuGbxDN6wtdEqjF4lisS3yWeEYUvWlkDNeZKTf"
    "gm8ZFu7mqFr%"
    "2FKMYae4lFH4JvGBVLgraghf09uQMZdabr5JfKC2q1zV3IgarOLLPMWllptvEo%"
    "2B1e7dkq5ZrLkip%2BKqa4lLk5ZvEY15INay9XIqXVGS%"
    "2BqsdcirzclYVa7hAPbQFVnJSaC9HCapavqjGXIjXvbNmSxi7eE4%"
    "2BsA21OumwuSQUJX1ZjLsVabqR6t7tUlrThhnjEC%"
    "2FFy6AKbCy45zdftmEutKm5UcSgHspY7XBEPVAFVHLoCUXPFkr3hi2wutba44QDr5iyeqQ3%"
    "2FiAccqOLQDhAV17pG0jZfUuZS5OJaGYiWF%2B2ypOV%"
    "2F4q5UQZtDu4G2xK10aeTlC1bhUslciQpYh7PSQtau8ErcVYZ4gXYDcUXLe1lrvBU%"
    "2B0VoutFRcWQWo4qwdTlYSr8Q9caDMwc3BDgl3xZpRb%"
    "2FORnuVCJHNlla2oOYmLQ8q7Ll6Ie6pgDaQKSCl8IF3WqAgPrbgU2VxpV1kje2EdoOWGlsOJ"
    "uKMd1g14OdjNp1YjNY%2B0m0s15kYgJVlaFxBVOETuAOK9eEELrDmUli%"
    "2Fo8oy94S4Xl2LzQGukEFU46RptQLy3BWWgHSBTvEp32eGRtjTjSriQBKLlShUPrSRcnK2qt"
    "IB4Zw3tQNRAbF5FB0vhoS57JFXzZmUtuLiy5gNlTTixlkgB8Y4byhAX0HJ4Y%"
    "2FcmWkjz0NrSaMNJ5EiNi3%"
    "2FSpPlIayqA3UBcIG5tQTuwBcQOJx3AsrSzxHJ4bKs9U5xoqWnK4U17%"
    "2BUzPFLQ4iQ3iRtxQC3gBK5xZJjOutcaSpeYjsUZqKFmGOLxIaflU1jI2ZzuLuLGuLe2yBlr"
    "LC1tdWg7ZmWal8KHeGtXG0gLLSdZyha%2BoKYdDl7WIGxpbI7lSicyLqFkH2rVZF%"
    "2BwUnymNXNu8WUkVLqSaB6IpIGWXF3Ft1UC6rRq3mhc7TRXgLS2lrKb5VEoz6nCSrtE2V6p4"
    "aMeQ8tJaxLU4nGU9o%2BXVTrMF%2BLBgjYqvSNkjL%"
    "2BDxhmut5tDb3CF1uwJoEdday6vMTHjVs7GA3g3QU8tXxZJc6Q23yhxWckPCtZW1nLgQ12KF"
    "F5Ed3pQ0U7yKp%2Fi6YM%"
    "2FI4dZOA3FRRdvhSmaWMxtxI3JzVlP8k9qsVFWdbVvTfENCjcytUoBW46XscE3DizLi1o6KQ"
    "4%2FDlZRsWSfBCt%2BSdHGrzGHFOtjFtUgNNJQR78Qjr%"
    "2BVwzV4I65SazPJzrQbKq6bl5kapU7bbRryXLo3c3LATYIfMEs3yc1bA44bScqumvJ21jLgr"
    "hHdSktNWkONR%"
    "2BLmULMnbpQm3pOWkZxHf0R7NKKykDr9iq3ptuexOuJQRZ5lCfE96K5Ct5iNpe118WQKxVeG"
    "CxnYDmUL8iUjb2%"
    "2BXmexIu9Di9XtgpxJ9wcehuOzwt1gJx4ynEM9K9tS5X7fLempP2dmnDczwjTlLYi%"
    "2FiCnHXSe9LWic9k3qvlRTltLU%2Bp2lE1sKUG8bm2DiNpNBpJu5vwwuEdLa%"
    "2FWy6p4JL27Dg%"
    "2B0pUBsQHxu67C1Vb2dpLlU5h3bG87aS0vNXWtJtip0bbjDhqgB8TkvH1g115qttnfDoW0oN"
    "e%"
    "2B1Rs0hlqVRc8cSmYP4XBUfUXHNlQ5tqzkpNaXmHVV4lVpq1NxjhYP43JqP2FwracOh7OZQD"
    "uXmRmu5sjMO75SWE%2FE5F4%"
    "2F09s5wI5abQ0rFoVxZNTes7e7wvy053NpwJj7n4kVCDt29teWypJHFOy0VJ6sN0CrK4dpak"
    "mv5pxQeEZ8rQ%2B9alnU2knyo2k64Ix4vh5I5sVNarqW3u8z%"
    "2F4mkeEZ8LrCxXtbfWu9t8qqQK0DKHVtEubrWm%2BZ9VPCS%"
    "2BJN1828oB4gqwalrFtUjNP3bzkPg7sdXAyhyssF4upWb5Z8c8Jv5QWmpgVRxUsGoulMw%"
    "2FPQqPiZ%2Fp8JGVOLQWKAW6%2BCcyF2qGD4gfibe2ead5lXEDpQAu0rv8r2WgtZxl1Twm%"
    "2Ftls1HxHK7HDjZV51VIgWmBlSeMKr%2BxseZYXq%"
    "2BUx8aY0MxrvVnUC4XNxgYtrJY15taMmNlAztd0lhxfW6MChC1rFY%"
    "2BLVjlwzKutVdfhE7xjKXEiX3CuHFzWG0lLycogUXnTtxuaws6DiMfFK09kZQ9K1VSvJ3oRH"
    "slIFWuGftdzQUoWzlYONinBILRdaC8TTYPO%2F3nBFnKxLG2um%"
    "2BKfXOrg6vBdLrvJSCm9SJpy0RtucrMRq1Zy1woUy0B4HbN60ex0uiEN0KLk1xZXs2paKW9F"
    "IqrJrzP%2Fs5k17tJz0GE%"
    "2FxohwulGElOUTmTRWl5oI4lKRRsTPhVpIdc6sl10IsFW9WXNpROPH0TkGAVnFpx5a63WSKN"
    "5HVXBKwc1btEffsNO8kvBObS5lZTnaUMXFYqbnUltwg75h%"
    "2FusMVATXleWW7qk1Xb8KLVfiKlsIlj9Sc1FhFtjITboSTlSp8QMCO5JU11bb1ZlQdIHL4ip"
    "rmktWROclqGlaWmvsSPiRAIy3lcAhk05vsWgfbU3xFVFyRWTUvSqqa2S7zHEFmRikt7yS18k"
    "xxFj6yY67UbNu86U6qIApPEUSasZb7Ek0DqXh5LHa4lDFWc6kd4uUpgsiaKR6pKQ61uHmsZr"
    "myk1ZxpQ1oeYoAzaG4ry1zsuXisVJxpeQdc60N2DxFgGckc1ePixdpHkjVjrnS0kpc6u5SwM"
    "tTBKxkybUJN3bUfCaulsMVTVvNP%2BmyNQVe7tjlE%"
    "2BJFb1mSLVfV9jaHHS2fiao15sqOd4pL29ArbxXvldV8TPwv6XVV6YXtGTefiiqaMRei2TFX"
    "KpzUONxKFWo%"
    "2BJt5J0ltlzQxfsCqimSpv86KmrHApBbXA2s2NuKPwMfFQvOELWgvsnEjVQMYtc2UXqjm0xI"
    "0yq%"
    "2FAx8T0JtyJz8DiekWpjxWoupRqqOamp5VJPsXJt9256wz3iW8oOt1xNaWah3NZJZK7UAg6H"
    "Lo%"
    "2B5tFPgke2SreUe8R1rO9xayTpALFaa2Z3mUhyo4qQ6I67MbLlsyyfFPeI71m7ey0orw2pL2"
    "56WuFILVHOI41mu1IyK3u0q28094nvCXQHLtqyF9Gq5tA7E4bAViRsrNW%"
    "2FCXeK3lDTVVoBI4ZIDVHFYpbTcyIbPiF%2FTSbPT3SUtl6qAuDl4W8UzxC%"
    "2Fz6CRciALUcijT4inil%2FV2p4pLtUDcwCol8xTxF8KlKg5VQGtb4jniz7UbWAcox%"
    "2BJJ4s%2B5OLiAVnuKJ4m%2FtuawBURbszxL%2FLF4OXgh9s7yNPHHqjisgVLLPE%"
    "2F8rXYD7UCrVsXzxN%2Bq4uAGrFj8gPhTXRzKwGprmh8Qf2rlot2AvSp%"
    "2BQvyl1nikAlprh58Qf0lqolGBarX8iPhLZWBVqnVsfkb8pTaHcru61PyM%"
    "2BEtrDq2UW8sPib%2FUChBvbIcfEn%"
    "2FKxWGrpeWnxJ9qVYDyVPgp8bfa2qRmmh8Tf21lq5qfE38uveE3%2FAdr385%2FSVd%"
    "2FMAAAAABJRU5ErkJggg%3D%3D";
} // namespace visualization
} // namespace adi